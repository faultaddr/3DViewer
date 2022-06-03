#ifndef _QT_GBK_H
#define _QT_GBK_H


#include <QString>
#include <QTextCodec>
#include <string>
using std::string;

class gbk
{
public:
	// QString(Unicode) -> std::string (gbk)
	static string FromUnicode(const QString& qstr)
	{
		QTextCodec* pCodec = QTextCodec::codecForName("gb2312");
		if(!pCodec) return "";	

		QByteArray arr = pCodec->fromUnicode(qstr);
		string cstr = arr.data();
		return cstr;
	}

	// std::string (gbk) -> QString(Unicode)
	static QString ToUnicode(const string& cstr)
	{
		QTextCodec* pCodec = QTextCodec::codecForName("gb2312");
		if(!pCodec) return "";

		QString qstr = pCodec->toUnicode(cstr.c_str(), cstr.length());
		return qstr;
	}

};


#endif

