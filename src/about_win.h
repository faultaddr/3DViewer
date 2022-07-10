#ifndef ABOUTWIN_H
#define ABOUTWIN_H

#include <QDialog>
#include <QWidget>

#include "gbk.h"
#include "ui_about_win.h"

class AboutWin : public QDialog {
  Q_OBJECT

 public:
  AboutWin(QWidget* parent = 0);
  ~AboutWin();

 private:
  Ui::AboutWin ui;
};

#endif  // ABOUTWIN_H
