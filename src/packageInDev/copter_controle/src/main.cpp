#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QIcon>
#include "personal_function.h"

class MyButton : public QWidget {
 public:
     MyButton(QWidget *parent = 0);
};

MyButton::MyButton(QWidget *parent): QWidget(parent) {

QPushButton *OffboardBtn3 = new QPushButton("Offboard", this); // создаем новую кнопку
OffboardBtn3->setGeometry(50, 40, 75, 30); // изменяем размеры кнопки в пикселях и помещаем на форму окна
QPushButton *ArmBtn2 = new QPushButton("Arm", this); // создаем новую кнопку
ArmBtn2->setGeometry(130, 40, 75, 30); // изменяем размеры кнопки в пикселях и помещаем на форму окна
QPushButton * DisarmBtn1= new QPushButton("Disarm", this); // создаем новую кнопку
DisarmBtn1->setGeometry(130, 80, 75, 30); // изменяем размеры кнопки в пикселях и помещаем на форму окна

connect(DisarmBtn1, &QPushButton::clicked, qApp, disarm);
connect(ArmBtn2, &QPushButton::clicked, qApp, arm);
connect(OffboardBtn3, &QPushButton::clicked, qApp, offboard);

}

void state_cb(const mavros_msgs::StateConstPtr &msg){  //принятие данных из топика
    cout << "1" << endl;
    // current_state = *msg; //записываются в переменную
}

int main(int argc, char **argv) {

ros::init(argc, argv, "control_copter_node");
ros::NodeHandle nh;

nh.subscribe<mavros_msgs::State>("/mavros/state", 1000, state_cb);
arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
// ros::spin();

QApplication app(argc, argv);
MyButton window;
window.resize(250, 150);
window.setWindowTitle("ControlCopter");
// window.setWindowIcon(QIcon("NII_RIPU.png"));
window.show();

return app.exec();    //app.exec()
// return 0;
}
