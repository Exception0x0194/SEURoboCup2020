#pragma once

#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>
#include <common/msg/game_data.hpp>
#include <common/msg/field_data.hpp>

enum TeamColor
{
    TEAM_RED,
    TEAM_BLUE
};

class TeamLabel: public QWidget
{
Q_OBJECT
public:
    TeamLabel(QString name, TeamColor color);
    QString Name()
    {
        return nameLabel->text();
    }
    void SetScore(int s)
    {
        scoreLabel->setText(QString::number(s));
    }
    void SetName(QString name)
    {
        nameLabel->setText(name);
    }
private:
    QLabel *colorLabel;
    QLabel *nameLabel;
    QLabel *scoreLabel;
};

class StartDlg: public QDialog
{
Q_OBJECT
public:
    StartDlg(std::string cfg);
    QString redName, blueName;
public slots:
    void OnStart();

private:
    QComboBox *redTeamBox, *blueTeamBox;
    QPushButton *startBtn;
};

class FieldDataSubscriber: public rclcpp::Node
{
public:
    FieldDataSubscriber(): Node("field_data_subscriber")
    {
        subscription_ = this->create_subscription<common::msg::FieldData>(
            "/sensor/field", 5, std::bind(&FieldDataSubscriber::topic_callback, this,
            std::placeholders::_1));
    }

    const common::msg::FieldData& GetData()
    {
        return fieldData_;
    }

private:
    void topic_callback(const common::msg::FieldData::SharedPtr msg)
    {
        fieldData_ = *msg;
    }

    common::msg::FieldData fieldData_;
    rclcpp::Subscription<common::msg::FieldData>::SharedPtr subscription_;
};

class GameDataPublisher : public rclcpp::Node
{
public:
    GameDataPublisher(): Node("game_publisher")
    {
        publisher_ = this->create_publisher<common::msg::GameData>("/sensor/game", 5);
    }

    void Publish(const common::msg::GameData& data)
    {
        publisher_->publish(data);
    }

    rclcpp::Publisher<common::msg::GameData>::SharedPtr publisher_;
};

class CtrlWindow: public QMainWindow
{
Q_OBJECT
public:
    CtrlWindow(QString red, QString blue);

public slots:
    void OnFTimer();
    void OnBtnInitClicked();
    void OnBtnReadyClicked();
    void OnBtnPlayClicked();
    void OnBtnPauseClicked();
    void OnBtnFinishClicked();

private:
    QLabel *timeLabel, *stateLabel, *infoLabel;
    TeamLabel *redTeam, *blueTeam;
    QTimer *fTimer;

    QPushButton *stateInitBtn, *stateReadyBtn, *statePlayBtn, *statePauseBtn, *stateFinishBtn;

    common::msg::GameData mGdata;
    common::msg::FieldData mFdata;
    std::shared_ptr<GameDataPublisher> gameDataPublisher;
    std::shared_ptr<FieldDataSubscriber> fieldDataSubscriber;

    QString startTime, goalTime;
    QString redName, blueName;
    bool paused;
    int mMs;
    int remainTime;
    int totalTime;
    const int mBasicTIme = 20;
};
