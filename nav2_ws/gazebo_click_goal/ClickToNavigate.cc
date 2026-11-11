#include <atomic>
#include <cmath>
#include <mutex>
#include <optional>
#include <QQuickItem>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/Plugin.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/RayQuery.hh>
#include <gz/transport/Node.hh>

class ClickToNavigate : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(QString status READ Status NOTIFY StatusChanged)
  public: QString Status() const { return this->status; }
  public: Q_INVOKABLE void SetEnabled(bool value) { this->enabled = value; }
  public: Q_INVOKABLE void Stop()
  {
    gz::msgs::Boolean msg;
    msg.set_data(true);
    this->stopPub.Publish(msg);
  }
  signals: void StatusChanged();

  protected: void LoadConfig(const tinyxml2::XMLElement *) override
  {
    this->title = "Boat navigation";
    this->goalPub = this->transport.Advertise<gz::msgs::Vector3d>("/nav/click_goal/world");
    this->stopPub = this->transport.Advertise<gz::msgs::Boolean>("/nav/click_goal/stop");
    this->transport.Subscribe("/nav/click_goal/status", &ClickToNavigate::OnStatus, this);
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
  }

  private: void OnStatus(const gz::msgs::StringMsg &msg)
  {
    const auto text = QString::fromStdString(msg.data());
    QMetaObject::invokeMethod(this, [this, text]() {
      this->status = text;
      emit this->StatusChanged();
    }, Qt::QueuedConnection);
  }

  protected: bool eventFilter(QObject *object, QEvent *event) override
  {
    if (event->type() == gz::gui::events::LeftClickOnScene::kType && this->enabled)
    {
      const auto &mouse = static_cast<gz::gui::events::LeftClickOnScene *>(event)->Mouse();
      if (!mouse.Dragging() && (mouse.Pos()-mouse.PressPos()).Length() < 4)
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->click = mouse.Pos();
      }
    }
    else if (event->type() == gz::gui::events::Render::kType)
    {
      std::optional<gz::math::Vector2i> pixel;
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        pixel.swap(this->click);
      }
      if (pixel) this->SendClick(*pixel);
    }
    return QObject::eventFilter(object, event);
  }

  private: void SendClick(const gz::math::Vector2i &pixel)
  {
    auto scene = gz::rendering::sceneFromFirstRenderEngine();
    if (!scene) return;
    gz::rendering::CameraPtr camera;
    for (unsigned i=0; i<scene->SensorCount(); ++i)
    {
      auto sensor = scene->SensorByIndex(i);
      if (sensor->HasUserData("user-camera"))
      {
        camera = std::dynamic_pointer_cast<gz::rendering::Camera>(sensor);
        if (camera) break;
      }
    }
    if (!camera || !camera->ImageWidth() || !camera->ImageHeight()) return;
    auto ray = scene->CreateRayQuery();
    const double nx = 2.*pixel.X()/camera->ImageWidth()-1.;
    const double ny = 1.-2.*pixel.Y()/camera->ImageHeight();
    ray->SetFromCamera(camera, gz::math::Vector2d(nx, ny));
    const auto origin = ray->Origin();
    const auto direction = ray->Direction();
    // Project onto mean water level z=0, never the seabed or mesh hit point.
    if (direction.Z() >= -1e-6) return;
    const double distance = -origin.Z()/direction.Z();
    if (!std::isfinite(distance) || distance <= 0 || distance > 2000) return;
    const auto point = origin+distance*direction;
    gz::msgs::Vector3d msg;
    msg.set_x(point.X());
    msg.set_y(point.Y());
    msg.set_z(0.);
    this->goalPub.Publish(msg);
  }

  private: gz::transport::Node transport;
  private: gz::transport::Node::Publisher goalPub, stopPub;
  private: std::atomic<bool> enabled{false};
  private: std::mutex mutex;
  private: std::optional<gz::math::Vector2i> click;
  private: QString status = "Waiting for navigation bridge...";
};

GZ_ADD_PLUGIN(ClickToNavigate, gz::gui::Plugin)
#include "ClickToNavigate.moc"
