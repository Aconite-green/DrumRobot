#include "../include/ChartHandler.hpp"

void ChartHandler::displayChartSlot()
{
    QDir directory("TuningData");
    QFileInfoList files = directory.entryInfoList(QStringList() << "*.csv", QDir::Files);

    std::sort(files.begin(), files.end(), [](const QFileInfo &a, const QFileInfo &b)
              { return a.created() > b.created(); });

    // Delete files if more than 5
    while (files.size() > 5)
    {
        QFile::remove(files.last().absoluteFilePath());
        files.removeLast();
    }

    if (files.isEmpty())
    {
        std::cerr << "No CSV files found in the TuningData directory." << std::endl;
        return;
    }

    for (int i = 0; i < std::min(files.size(), 2); ++i)
    {

        QMainWindow *window = new QMainWindow();
        QSplitter *splitter = new QSplitter();

        QString filePath = files[i].absoluteFilePath();
        std::string fileName = files[i].fileName().toStdString();
        QString qFileName = QString::fromStdString(fileName);
        

        // Extract Kp and Kd values from the filename
        std::regex kpKdRegex("kp_([0-9.]+)_kd_([0-9.]+)");
        std::smatch kpKdMatch;
        QString chartTitle;
        if (std::regex_search(fileName, kpKdMatch, kpKdRegex) && kpKdMatch.size() > 2)
        {
            chartTitle = "<b><font size='5'>Kp : " + QString::fromStdString(kpKdMatch.str(1)) +
                         " Kd : " + QString::fromStdString(kpKdMatch.str(2)) + "</font></b>";
        }
        else
        {
            chartTitle = "<b>Chart</b>";
        }

        std::ifstream file(filePath.toStdString());

        if (!file.is_open())
        {
            std::cerr << "Failed to open file: " << fileName << std::endl;
            return;
        }

        QLineSeries *seriesPDes = new QLineSeries();
        seriesPDes->setName("P_des");
        QLineSeries *seriesPAct = new QLineSeries();
        seriesPAct->setName("P_act");
        QLineSeries *seriesTffDes = new QLineSeries();
        seriesTffDes->setName("Tff_des");
        QLineSeries *seriesTffAct = new QLineSeries();
        seriesTffAct->setName("Tff_act");

        std::string line;
        getline(file, line); // Skip header line

        int time = 0;
        int lineCount = 0;
        while (getline(file, line))
        {
            std::stringstream ss(line);
            std::string value;
            std::vector<float> values;

            while (getline(ss, value, ','))
            {
                values.push_back(std::stof(value));
            }

            if (values.size() != 5)
            {
                std::cerr << "Invalid line format: " << line << std::endl;
                continue;
            }

            seriesPDes->append(time, values[1]);
            seriesPAct->append(time, values[2]);
            seriesTffDes->append(time, values[3]);
            seriesTffAct->append(time, values[4]);
            time += 5; // Increment time by 5ms for each line
            lineCount++;
        }

        

        // Create new axes and set their properties
        QValueAxis *axisXPosition = new QValueAxis();
        axisXPosition->setTitleText("Time [ms]");
        QValueAxis *axisYPosition = new QValueAxis();
        axisYPosition->setTitleText("Position [rad]");
        axisYPosition->setLabelFormat("%0.2f");
        axisYPosition->setRange(-2.0, 2.0); // Adjust the range as needed

        QValueAxis *axisXTorque = new QValueAxis();
        axisXTorque->setTitleText("Time [ms]");
        QValueAxis *axisYTorque = new QValueAxis();
        axisYTorque->setTitleText("Torque [N/m]");
        axisYTorque->setLabelFormat("%0.2f");
        axisYTorque->setRange(-10.0, 10.0); // Adjust the range as needed

        // Configure the Position chart
        QChart *chartP = new QChart();
        chartP->setTitle(chartTitle);
        chartP->addAxis(axisXPosition, Qt::AlignBottom);
        chartP->addAxis(axisYPosition, Qt::AlignLeft);
        chartP->addSeries(seriesPDes);
        chartP->addSeries(seriesPAct);
        seriesPDes->attachAxis(axisXPosition);
        seriesPDes->attachAxis(axisYPosition);
        seriesPAct->attachAxis(axisXPosition);
        seriesPAct->attachAxis(axisYPosition);

        // Configure the Torque chart
        QChart *chartTff = new QChart();
        chartTff->setTitle(chartTitle);
        chartTff->addAxis(axisXTorque, Qt::AlignBottom);
        chartTff->addAxis(axisYTorque, Qt::AlignLeft);
        chartTff->addSeries(seriesTffDes);
        chartTff->addSeries(seriesTffAct);
        seriesTffDes->attachAxis(axisXTorque);
        seriesTffDes->attachAxis(axisYTorque);
        seriesTffAct->attachAxis(axisXTorque);
        seriesTffAct->attachAxis(axisYTorque);

        // Create chart views and configure them
        QChartView *chartViewP = new QChartView(chartP);
        chartViewP->setRenderHint(QPainter::Antialiasing);
        QChartView *chartViewTff = new QChartView(chartTff);
        chartViewTff->setRenderHint(QPainter::Antialiasing);

        // Add chart views to the splitter
        splitter->addWidget(chartViewP);
        splitter->addWidget(chartViewTff);

        // Setup the main window
        window->setCentralWidget(splitter);
        window->resize(1280, 720); // Adjust the window size as needed
        window->show();
    }
}
