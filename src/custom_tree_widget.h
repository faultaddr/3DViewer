//
// Created by pyy on 22-6-3.
//

#ifndef CLOUDVIEWER_SRC_CUSTOM_TREE_WIDGET_H_
#define CLOUDVIEWER_SRC_CUSTOM_TREE_WIDGET_H_

#include <QtWidgets/QWidget>
#include <QLabel>
#include <QHBoxLayout>
#include <QPainter>
#include <QListWidget>
#include <QScrollBar>
#include <QMouseEvent>
#include <QScrollArea>
#include <QTimer>

#define ITEM_HEIGHT 40    // item高度;

// 传感器数据结构;
struct SensorDataStruct {
  QColor alarmColor;
  QString sensorName;
  float sensorNumber;
};

// 节点数据结构;
struct NodeDataStruct {
  QString strNodeName;
  int strNodeCount;
  QColor numberColor;
  QList<SensorDataStruct> sensorDataList;
};

class SensorItemWidget : public QWidget {
 public:
  SensorItemWidget(QWidget *parent = NULL)
      : QWidget(parent) {
    initWidget();
    this->setFixedSize(QSize(250, ITEM_HEIGHT));
    this->setStyleSheet(".QWidget:hover{background:rgba(200, 200, 200, 150);}");
  }

  // 设置传感器信息;
  void setSensorInfo(QColor alarmColor, QString sensorName, float sensorNumber) {
    m_colorWidget->setStyleSheet(QString("QWidget{border-radius:6px;background:rgb(%1, %2, %3);}").arg(alarmColor.red()).arg(
        alarmColor.green()).arg(alarmColor.blue()));

    m_sensorLabel->setText(sensorName);
    m_sensorLabel->setScaledContents(true);
    m_numberLabel->setText(QString::number(sensorNumber));
    m_numberLabel->setScaledContents(true);
  }

 private:
  void initWidget() {
    m_colorWidget = new QWidget;
    m_colorWidget->setFixedSize(QSize(12, 12));

    m_sensorLabel = new QLabel;

    m_numberLabel = new QLabel;

    QHBoxLayout *hLayout = new QHBoxLayout(this);
    hLayout->addWidget(m_colorWidget);
    hLayout->addWidget(m_sensorLabel);
    hLayout->addWidget(m_numberLabel);
    hLayout->addStretch();
    hLayout->setSpacing(15);
    hLayout->setContentsMargins(20, 0, 0, 0);
  }

 private:
  QWidget *m_colorWidget;
  QLabel *m_sensorLabel;
  QLabel *m_numberLabel;
};

class SencorListWidget : public QWidget {
 Q_OBJECT

 public:
  SencorListWidget(QWidget *parent = NULL)
      : QWidget(parent), m_isFolded(true) {
    initWidget();
    this->setWindowFlags(Qt::FramelessWindowHint);

    this->setStyleSheet("QListWidget{border:none;border-bottom:1px solid gray;}");
  }

  // 设置标题栏信息;
  void setTitleInfo(QString strTitle, int count, QColor numberColor) {
    m_titleLabel->setText(strTitle);
    m_numberLabel->setText(QString::number(count));
    m_numberLabel->setStyleSheet(QString("color:white;border-radius:8px;background:rgb(%1, %2, %3);").arg(numberColor.red()).arg(
        numberColor.green()).arg(numberColor.blue()));
  }

  // 添加传感器子项;
  void addSensorItem(QColor alarmColor, QString sensorName, float sensorNumber) {
    QListWidgetItem *item = new QListWidgetItem;
    item->setSizeHint(QSize(250, ITEM_HEIGHT));
    m_listWidget->addItem(item);
    SensorItemWidget *itemWidget = new SensorItemWidget;
    itemWidget->setSensorInfo(alarmColor, sensorName, sensorNumber);
    m_listWidget->setItemWidget(item, itemWidget);
  }

  // 更新传感器某一项状态;
  void updateSensorItem(int rowCount, QColor alarmColor, QString sensorName, float sensorNumber) {
    QListWidgetItem *item = m_listWidget->item(rowCount);
    if (item != NULL) {
      SensorItemWidget *itemWidget = static_cast<SensorItemWidget *>(m_listWidget->itemWidget(item));
      itemWidget->setSensorInfo(alarmColor, sensorName, sensorNumber);
    }
  }

 private:
  void initTitleBackWidget() {
    m_titleBackWidget = new QWidget;
    m_titleBackWidget->setFixedSize(QSize(250, 40));
    m_titleBackWidget->installEventFilter(this);
    m_titleBackWidget->setStyleSheet(".QWidget{border-bottom:1px solid gray;}\
                                            .QWidget:hover{background:rgba(200, 200, 200, 200);}");

    m_foldStateLabel = new QLabel;
    m_foldStateLabel->setFixedSize(QSize(20, 20));
    m_foldStateLabel->setPixmap(QIcon(":/Resources/Folded.png").pixmap(m_foldStateLabel->size()));

    m_titleLabel = new QLabel;
    m_titleLabel->setStyleSheet("font-size:16px;font-weight:bold;");

    m_numberLabel = new QLabel;
    m_numberLabel->setFixedSize(QSize(16, 16));
    m_numberLabel->setAlignment(Qt::AlignCenter);

    QHBoxLayout *hButtonLayout = new QHBoxLayout;
    hButtonLayout->addWidget(m_numberLabel);
    hButtonLayout->setContentsMargins(0, 0, 0, 10);

    m_foldTagLabel = new QLabel;
    m_foldTagLabel->setFixedSize(QSize(20, 20));
    m_foldTagLabel->setPixmap(QIcon(":/Resources/EnableFold.png").pixmap(m_foldTagLabel->size()));

    QHBoxLayout *hTitleLayout = new QHBoxLayout(m_titleBackWidget);
    hTitleLayout->addWidget(m_foldStateLabel);
    hTitleLayout->addWidget(m_titleLabel);
    hTitleLayout->addLayout(hButtonLayout);
    hTitleLayout->addStretch();
    hTitleLayout->addWidget(m_foldTagLabel);
    hTitleLayout->setSpacing(5);
    hTitleLayout->setContentsMargins(5, 0, 25, 0);
  }

  void initWidget() {
    initTitleBackWidget();

    m_listWidget = new QListWidget;
    m_listWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_listWidget->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_listWidget->setVisible(false);

    QVBoxLayout *vMainLayout = new QVBoxLayout(this);
    vMainLayout->addWidget(m_titleBackWidget);
    vMainLayout->addWidget(m_listWidget);
    vMainLayout->addStretch();
    vMainLayout->setSpacing(0);
    vMainLayout->setMargin(0);
  }

  bool eventFilter(QObject *watched, QEvent *event) override {
    if (watched == m_titleBackWidget) {
      if (event->type() == QEvent::MouseButtonRelease) {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent *>(event);
        if (mouseEvent->button() == Qt::LeftButton) {
          // 节点点击进行展开收缩;
          if (m_isFolded) {
            m_foldStateLabel->setPixmap(QIcon(":/Resources/UnFolded.png").pixmap(m_foldStateLabel->size()));
            m_listWidget->setVisible(true);
            m_listWidget->setFixedHeight(m_listWidget->count() * ITEM_HEIGHT + 10);
            this->setFixedHeight(m_titleBackWidget->height() + m_listWidget->height());
            emit signalNodeFoldChanged(m_titleBackWidget->height() + m_listWidget->height());
          } else {
            m_foldStateLabel->setPixmap(QIcon(":/Resources/Folded.png").pixmap(m_foldStateLabel->size()));
            m_listWidget->setVisible(false);
            this->setFixedHeight(m_titleBackWidget->height());
            emit signalNodeFoldChanged(m_titleBackWidget->height());
          }
          m_isFolded = !m_isFolded;
        }
      }
      return true;
    }
    return false;
  }

 signals:
  void signalNodeFoldChanged(int itemHeight);

 private:
  QLabel *m_foldStateLabel;
  QLabel *m_titleLabel;
  QLabel *m_numberLabel;
  QLabel *m_foldTagLabel;

  QWidget *m_titleBackWidget;
  QListWidget *m_listWidget;

  bool m_isFolded;
};

class CustomTreeWidget : public QWidget {
 Q_OBJECT

 public:
  CustomTreeWidget(QWidget *parent = Q_NULLPTR);

 private:
  void initWidget();

 private slots:
  void onNodeFoldChanged();

  void onUpdateData();

 private:

  QScrollArea *m_scrollArea;

  QList<NodeDataStruct> m_nodeDataList;

  QList<SencorListWidget *> m_nodeWidgetList;

  QWidget *m_sensorBackWidget;

  // 刷新数据时钟;
  QTimer m_refreshTimer;
  int m_refreshIndex;
};

#endif //CLOUDVIEWER_SRC_CUSTOM_TREE_WIDGET_H_
