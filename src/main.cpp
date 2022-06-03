#include "CloudViewer.h"
#include <QtWidgets/QApplication>
#include "glog/logging.h"
#include <iomanip>
#include <iostream>
void CustomPrefix(std::ostream &s, const google::LogMessageInfo &l, void *) {
  s << "["
    << l.severity
    << "] "
    << std::setw(4) << 1900 + l.time.year()
    << std::setw(2) << 1 + l.time.month()
    << std::setw(2) << l.time.day()
    << ' '
    << std::setw(2) << l.time.hour() << ':'
    << std::setw(2) << l.time.min() << ':'
    << std::setw(2) << l.time.sec() << "."
    << std::setw(6) << l.time.usec()
    << ' '
    << std::setfill(' ') << std::setw(5)
    << l.thread_id << std::setfill('0')
    << ' '
    << l.filename << ':' << l.line_number;
}
int main(int argc, char *argv[]) {

  google::InitGoogleLogging(argv[0], &CustomPrefix);
  FLAGS_logtostderr = false;
  FLAGS_alsologtostderr= false;
  FLAGS_stderrthreshold = google::GLOG_ERROR;
  FLAGS_log_prefix = true;
  FLAGS_logbufsecs = 0;
  FLAGS_max_log_size = 10;
  FLAGS_stop_logging_if_full_disk=true;
  google::SetStderrLogging(google::GLOG_INFO);
  google::SetLogDestination(google::GLOG_INFO,"log_");
  google::SetLogDestination(google::GLOG_WARNING,"log_");
  google::SetLogDestination(google::GLOG_ERROR,"log_");
  google::SetLogDestination(google::GLOG_FATAL,"log_");

  google::SetLogFilenameExtension(".txt");
  LOG(INFO) << "---------------------start log---------------------";

  QApplication a(argc, argv);

  CloudViewer w;
  w.show();
  return a.exec();
}
