#ifndef GUI_POSITIONS_H
#define GUI_POSITIONS_H

#include <QDialog>
#include <QTimer>
#include <QString>

#include <variable.h>

namespace Ui {
class gui_positions;
}

class gui_positions : public QDialog
{
    Q_OBJECT

public:
    explicit gui_positions(QWidget *parent = nullptr);
    ~gui_positions();

private:
    Ui::gui_positions *ui;

    void mainloop();

    QString darkgrey="background-color: rgba(65, 65, 65, 0);\ncolor: rgb(255, 255, 255);";
    QString green="background-color: rgba(170, 255, 0, 0);\ncolor: rgb(0, 0, 0);";
    QString red="background-color: rgba(242, 0, 0, 0);\ncolor: rgb(0, 0, 0);";
    QString orange="background-color: rgba(255, 170, 0, 0);\ncolor: rgb(0, 0, 0);";

};

#endif // GUI_POSITIONS_H
