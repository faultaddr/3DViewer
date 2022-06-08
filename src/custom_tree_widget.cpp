#include "custom_tree_widget.h"

#include <QByteArray>
#include <QDebug>
#include <QDrag>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QImage>
#include <QLabel>
#include <QMimeData>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>

CustomTreeWidget::CustomTreeWidget(QWidget* parent) : QTreeWidget(parent) {
  this->installEventFilter(this);
  this->setMouseTracking(true);
  this->setAcceptDrops(true);
  m_bNewTreeState = false;
}

void CustomTreeWidget::setNewTreeState(bool state) {
  m_bNewTreeState = state;
}

void CustomTreeWidget::dragEnterEvent(QDragEnterEvent* event) {
  if (event->mimeData()->hasFormat("application/x-dnditemdata")) {
    if (event->source() == this) {
      event->setDropAction(Qt::MoveAction);
      event->accept();
    } else {
      event->acceptProposedAction();
    }
  } else {
    event->ignore();
  }
}

void CustomTreeWidget::dragMoveEvent(QDragMoveEvent* event) {
  if (event->mimeData()->hasFormat("application/x-dnditemdata")) {
    if (event->source() == this) {
      event->setDropAction(Qt::MoveAction);
      event->accept();
    } else {
      event->acceptProposedAction();
    }
  } else {
    event->ignore();
  }
}

void CustomTreeWidget::dropEvent(QDropEvent* event) {
  //    if (event->mimeData()->hasFormat("application/x-dnditemdata")) {
  //        QByteArray itemData =
  //        event->mimeData()->data("application/x-dnditemdata"); QDataStream
  //        dataStream(&itemData, QIODevice::ReadOnly);

  //        QPixmap pixmap;
  //        QPoint offset;
  //        dataStream >> pixmap >> offset;

  //        QLabel *newIcon = new QLabel(this);
  //        newIcon->setPixmap(pixmap);
  //        newIcon->move(event->pos() /*- offset*/);
  //        newIcon->show();
  //        newIcon->setAttribute(Qt::WA_DeleteOnClose);

  //        if (event->source() == this) {
  //            event->setDropAction(Qt::MoveAction);
  //            event->accept();
  //        } else {
  //            event->acceptProposedAction();
  //        }
  //    } else {
  //        event->ignore();
  //    }

  //    qDebug() << "-------dropEvent";
  //    if(m_pNewTreeWidget == nullptr) {
  //        qDebug() << "-------2";
  //        m_pNewTreeWidget = new NewTreeWidget;
  //        m_pNewTreeWidget->resize(200, 300);
  //        m_pNewTreeWidget->move(mapToGlobal(event->pos()));
  //        m_pNewTreeWidget->show();
  //    }
}

void CustomTreeWidget::dragLeaveEvent(QDragLeaveEvent* event) {
  qDebug() << "-------dragLeaveEvent";

  Q_UNUSED(event);

  QList<QString> listItemText = getSelectedItemsText();
  if (!m_bNewTreeState) {
    if (listItemText.count()) {
      emit signalCreateNewTree(listItemText);
    }
  } else {
    if (listItemText.count()) {
      emit signalNewItemText(listItemText);
    }
  }
}

void CustomTreeWidget::mouseReleaseEvent(QMouseEvent* event) {
  is_pressed = false;
  QTreeWidget::mouseReleaseEvent(event);
  qDebug() << "-------mouseReleaseEvent---------------";
}

// void DragTreeWidget::mouseMoveEvent(QMouseEvent *event)
//{
//    static int i = 0;
//    qDebug() << "-------mouseMoveEvent" << i++;
//}

bool CustomTreeWidget::eventFilter(QObject* target, QEvent* event) {
  //    if(event->type() == QEvent::MouseMove)
  //    {
  //       // This piece of code is never called
  //        qDebug() << "-------mouseMoveEvent";
  //    }
  return QTreeWidget::eventFilter(target, event);
}

QList<QString> CustomTreeWidget::getSelectedItemsText() {
  QList<QString> listItemText;
  QList<QTreeWidgetItem*> listItem = this->selectedItems();
  if (listItem.count()) {
    foreach (QTreeWidgetItem* item, listItem) {
      QString text = item->text(0);
      if (item->childCount() == 0)
        listItemText.append(text);
    }
  }
  return listItemText;
}

void CustomTreeWidget::mousePressEvent(QMouseEvent* event) {
  is_pressed = true;
  index_list_ = this->selectedIndexes();
  QTreeWidget::mousePressEvent(event);
  QList<QString> listText = getSelectedItemsText();
  if (listText.count() == 0) {
    return;
  }

  QImage image(30, 10, QImage::Format_ARGB32);
  image.fill(QColor(47, 168, 236, 150));
  QPixmap pixmap = QPixmap::fromImage(image);
  // pixmap.load(Global::GetImagePath() + "hover.png");

  QByteArray itemData;
  QDataStream dataStream(&itemData, QIODevice::WriteOnly);
  dataStream << pixmap
             << QPoint(event->pos() /*- child->pos()*/ +
                       QPoint(pixmap.width() / 2, pixmap.height() / 2));

  QMimeData* mimeData = new QMimeData;
  mimeData->setData("application/x-dnditemdata", itemData);

  QPixmap tempPixmap = pixmap;
  QPainter painter;
  painter.begin(&tempPixmap);
  // painter.fillRect(pixmap.rect(), QColor(47, 168, 236, 150));
  if (m_bNewTreeState) {
    QFont font = painter.font();
    font.setPixelSize(16);
    painter.setFont(font);
    QPen pen = painter.pen();
    pen.setStyle(Qt::DotLine);
    pen.setColor(QColor("#2FEC2F"));
    painter.setPen(pen);

    for (int i = 0; i < listText.size(); i++) {
      const QRect rectangle = QRect(10, 10 + 40 * i, pixmap.width() - 10, 50);
      QRect boundingRect;
      painter.drawText(rectangle, 0, listText.at(i), &boundingRect);
    }
    auto index_list = this->selectedIndexes();
    for (auto& index : index_list) {
    }
  }
  painter.end();

  QDrag* drag = new QDrag(this);
  drag->setMimeData(mimeData);
  drag->setPixmap(tempPixmap);
  drag->setHotSpot(/*mapToGlobal(event->pos())*/ QPoint(pixmap.width() / 2,
                                                        pixmap.height() / 2));

  if (drag->exec(Qt::CopyAction | Qt::MoveAction, Qt::CopyAction) ==
      Qt::MoveAction) {
    //        child->close();
  } else {
    //        child->show();
    //        child->setPixmap(pixmap);
  }
}
bool CustomTreeWidget::IsMousePressedFromDataTree() {
  return is_pressed;
}
