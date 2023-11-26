// ChartHandler.hpp
#ifndef CHARTHANDLER_HPP
#define CHARTHANDLER_HPP

#include <QDir>
#include <QFileInfoList>
#include <QStringList>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QChartView>
#include <QtWidgets/QMainWindow>
#include <QtCharts/QValueAxis>
#include <QSplitter> 
#include <QDateTime>

#include <fstream>
#include <sstream>
#include <iostream>
#include <regex>

using namespace QtCharts;
using namespace std;

class ChartHandler : public QObject
{
    Q_OBJECT

public:
    ChartHandler(QObject *parent = nullptr) : QObject(parent) {}

signals:
    void displayChartSignal();
    void requestQuit();

public slots:
    void displayChartSlot();
};
#endif // CHARTHANDLER_HPP

