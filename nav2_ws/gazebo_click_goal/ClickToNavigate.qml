import QtQuick 2.12
import QtQuick.Controls 2.5
import QtQuick.Layouts 1.12

ColumnLayout {
  width: 300
  Layout.minimumWidth: 300
  Layout.minimumHeight: 430
  spacing: 12
  Label {
    text: "Choose a water destination"
    font.bold: true
    wrapMode: Text.WordWrap
    Layout.fillWidth: true
  }
  Switch {
    text: "Click to navigate"
    onToggled: ClickToNavigate.SetEnabled(checked)
  }
  Label {
    text: "Enable, then left-click the water. Drag to move the camera. A new click redirects the boat."
    wrapMode: Text.WordWrap
    Layout.fillWidth: true
  }
  Label {
    text: ClickToNavigate.status
    wrapMode: Text.WordWrap
    Layout.fillWidth: true
  }
  Button {
    text: "Stop boat"
    onClicked: ClickToNavigate.Stop()
    Layout.fillWidth: true
  }
  Label {
    text: "Each goal requests arming. Nav2 plans around observed obstacles; sensor faults stop thrust."
    wrapMode: Text.WordWrap
    Layout.fillWidth: true
  }
  Item { Layout.fillHeight: true }
}
