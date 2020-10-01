#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "ctrlwindow.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    std::string cfg = "teams.cfg";
    if(argc > 1) {
        cfg = std::string(argv[1]);
    }
    StartDlg sdlg(cfg);
    sdlg.exec();
    if(sdlg.redName.size() > 0) {
        CtrlWindow foo(sdlg.redName, sdlg.blueName);
        foo.show();
        return app.exec();
    }
    return 0;
}