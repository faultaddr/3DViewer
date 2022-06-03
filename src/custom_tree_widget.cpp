//
// Created by pyy on 22-6-3.
//

#include "custom_tree_widget.h"

CustomTreeWidget::CustomTreeWidget(QWidget *parent)
    : QWidget(parent), m_refreshIndex(0) {
  initWidget();

  this->setFixedSize(QSize(250, 600));

  m_refreshTimer.setInterval(500);
  connect(&m_refreshTimer, &QTimer::timeout, this, &CustomTreeWidget::onUpdateData);
  m_refreshTimer.start();

  this->setStyleSheet("*{font-family:Microsoft YaHei;}\
           QScrollBar::vertical {\
              background:rgb(226,222,221);\
              border:none;\
              width: 5px;\
              margin:0px;\
           }\
           QScrollBar::handle:vertical {\
              background: rgb(192,192,192);\
              border-radius:1px;\
              min-height: 20px;\
              width:5px;\
          }\
          QScrollBar::add-line:vertical {\
              height:0px;\
          }\
          QScrollBar::sub-line:vertical {\
              height:0px;\
          }\
          QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {\
             background:transparent;\
          }");
}

void CustomTreeWidget::initWidget() {
  m_scrollArea = new QScrollArea;
  m_scrollArea->setFixedWidth(250);
  m_scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  m_scrollArea->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  m_scrollArea->setStyleSheet(".QScrollArea{background:white;}");

  for (int i = 0; i < 4; i++) {
    NodeDataStruct nodeData;
    nodeData.numberColor = Qt::red;
    nodeData.strNodeCount = 6;
    nodeData.strNodeName = QString("Node-%1").arg(i);
    for (int j = 0; j < 6; j++) {
      SensorDataStruct sensorData;
      sensorData.alarmColor = Qt::blue;
      sensorData.sensorName = QString("Sensor-%1").arg(j);
      sensorData.sensorNumber = j;
      nodeData.sensorDataList.append(sensorData);
    }
    m_nodeDataList.append(nodeData);
  }

  m_sensorBackWidget = new QWidget;
  m_sensorBackWidget->setFixedWidth(250);
  m_sensorBackWidget->setStyleSheet(".QWidget{background:white;}");
  QVBoxLayout *vBackWidget = new QVBoxLayout(m_sensorBackWidget);
  vBackWidget->setSpacing(0);
  vBackWidget->setMargin(0);
  // 根据已经造好的数据布局界面;
  for (int i = 0; i < m_nodeDataList.size(); i++) {
    NodeDataStruct nodeData = m_nodeDataList[i];
    SencorListWidget *sencorListWidget = new SencorListWidget;
    connect(sencorListWidget, &SencorListWidget::signalNodeFoldChanged, this, &CustomTreeWidget::onNodeFoldChanged);
    // 先设置节点标题信息;
    sencorListWidget->setTitleInfo(nodeData.strNodeName, nodeData.strNodeCount, nodeData.numberColor);
    // 为每个节点添加数据;
    for (int j = 0; j < nodeData.sensorDataList.count(); j++) {
      SensorDataStruct sensorData = nodeData.sensorDataList[j];
      sencorListWidget->addSensorItem(sensorData.alarmColor, sensorData.sensorName, sensorData.sensorNumber);
    }

    vBackWidget->addWidget(sencorListWidget);
    m_nodeWidgetList.append(sencorListWidget);
  }

  m_scrollArea->setWidget(m_sensorBackWidget);

  QHBoxLayout *hMainLayout = new QHBoxLayout(this);
  hMainLayout->addWidget(m_scrollArea);
  hMainLayout->setMargin(0);
}

void CustomTreeWidget::onNodeFoldChanged() {
  int height = 0;
  for (int i = 0; i < m_nodeWidgetList.count(); i++) {
    height += m_nodeWidgetList[i]->height();
  }
  m_sensorBackWidget->setFixedHeight(height);
}

// 刷新数据;
void CustomTreeWidget::onUpdateData() {
  if (m_refreshIndex >= m_nodeDataList.count()) {
    m_refreshIndex = 0;
  }

  // 这里500ms更新一个节点的数据;
  SencorListWidget *sencorListWidget = m_nodeWidgetList[m_refreshIndex];
  NodeDataStruct nodeData = m_nodeDataList[m_refreshIndex];
  nodeData.numberColor = QColor(rand() % 256, rand() % 256, rand() % 256);
  sencorListWidget->setTitleInfo(nodeData.strNodeName, nodeData.strNodeCount, nodeData.numberColor);

  for (int j = 0; j < 6; j++) {
    SensorDataStruct sensorData = nodeData.sensorDataList[j];
    sensorData.alarmColor = nodeData.numberColor = QColor(rand() % 256, rand() % 256, rand() % 256);
    sencorListWidget->updateSensorItem(j, sensorData.alarmColor, sensorData.sensorName, sensorData.sensorNumber);
  }

  m_refreshIndex++;
}

