//
// Created by robolab on 30/3/21.
//

#include <dsr/api/dsr_api.h>
#include <dsr/gui/dsr_gui.h>
#include <qapplication.h>



int main( int argc, char **argv )
{
    QApplication a( argc, argv );

    int rvalue = a.exec();
    return rvalue;
}