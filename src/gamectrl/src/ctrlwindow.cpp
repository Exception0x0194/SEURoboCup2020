#include "ctrlwindow.hpp"
#include <map>
#include <fstream>

using namespace std;

map<int, string> states = {
    {common::msg::GameData::STATE_INIT, "Init"},
    {common::msg::GameData::STATE_READY, "Ready"},
    {common::msg::GameData::STATE_PLAY, "Play"},
    {common::msg::GameData::STATE_PAUSE, "Pause"},
    {common::msg::GameData::STATE_END, "End"},
};


TeamLabel::TeamLabel(QString name, TeamColor color)
{
    colorLabel = new QLabel();
    colorLabel->setFixedWidth(16);
    QString cstr = color == TEAM_RED ? "background-color:red;" : "background-color:blue";
    colorLabel->setStyleSheet(cstr);
    nameLabel = new QLabel(name);
    nameLabel->setStyleSheet("font-size:20px;");
    scoreLabel = new QLabel("0");
    scoreLabel->setFixedWidth(30);
    scoreLabel->setAlignment(Qt::AlignCenter);
    scoreLabel->setStyleSheet("font-size:20px;");

    QHBoxLayout *lay = new QHBoxLayout();
    lay->addWidget(colorLabel);
    lay->addWidget(nameLabel);
    lay->addWidget(scoreLabel);
    this->setLayout(lay);
}

StartDlg::StartDlg(std::string cfg)
{
    string line;
    QStringList teams;
    ifstream ifs(cfg);
    if(ifs) {
        while(getline(ifs, line)) {
            if(!line.empty() && line.back() == '\n') {
                line.pop_back();
            }
            if(!line.empty()) {
                teams << QString::fromStdString(line);
            }
            line.clear();
        }
        ifs.close();
    }
    QVBoxLayout *leftLayout, *rightLayout;
    leftLayout = new QVBoxLayout;
    rightLayout = new QVBoxLayout;
    QLabel *redLabel = new QLabel();
    redLabel->setStyleSheet("background-color: red;");
    QLabel *blueLabel = new QLabel();
    blueLabel->setStyleSheet("background-color: blue;");
    redTeamBox = new QComboBox();
    blueTeamBox = new QComboBox();
    redTeamBox->addItems(teams);
    blueTeamBox->addItems(teams);
    leftLayout->addWidget(redLabel);
    leftLayout->addWidget(redTeamBox);
    rightLayout->addWidget(blueLabel);
    rightLayout->addWidget(blueTeamBox);

    startBtn = new QPushButton("Start");
    connect(startBtn, &QPushButton::clicked, this, &StartDlg::OnStart);

    QVBoxLayout *mainLayout = new QVBoxLayout;
    QHBoxLayout *upLayout = new QHBoxLayout;
    upLayout->addLayout(leftLayout);
    upLayout->addLayout(rightLayout);
    mainLayout->addLayout(upLayout);
    mainLayout->addWidget(startBtn);
    setLayout(mainLayout);
}

void StartDlg::OnStart()
{
    redName = redTeamBox->currentText();
    blueName = blueTeamBox->currentText();

    if(redName.size() == 0 || blueName.size() == 0) {
        return;
    }
    if(redName == blueName) {
        QMessageBox::warning(this, "Error", "Two teams is the same!");
        return;
    }
    this->close();
}

CtrlWindow::CtrlWindow(QString red, QString blue)
{
    redName = red;
    blueName = blue;
    totalTime = 60 * 10;

    QHBoxLayout *mainLayout = new QHBoxLayout();
    QVBoxLayout *leftLayout, *rightLayout;
    leftLayout = new QVBoxLayout();
    rightLayout = new QVBoxLayout();

    timeLabel = new QLabel(QString::number(totalTime));
    timeLabel->setAlignment(Qt::AlignCenter);
    timeLabel->setStyleSheet("font-size:36px;");
    stateLabel = new QLabel("Init");
    stateLabel->setAlignment(Qt::AlignCenter);
    stateLabel->setStyleSheet("font-size:36px;");
    infoLabel = new QLabel();

    redTeam = new TeamLabel(red, TEAM_RED);
    blueTeam = new TeamLabel(blue, TEAM_BLUE);


    leftLayout->addWidget(timeLabel);
    leftLayout->addWidget(stateLabel);
    leftLayout->addWidget(infoLabel);
    leftLayout->addWidget(redTeam);
    leftLayout->addWidget(blueTeam);

    stateInitBtn = new QPushButton("Init");
    stateInitBtn->setMinimumHeight(40);
    stateInitBtn->setStyleSheet("font-size:20px;");
    rightLayout->addWidget(stateInitBtn);
    stateReadyBtn = new QPushButton("Ready");
    stateReadyBtn->setMinimumHeight(40);
    stateReadyBtn->setStyleSheet("font-size:20px;");
    rightLayout->addWidget(stateReadyBtn);
    statePlayBtn = new QPushButton("Play");
    statePlayBtn->setMinimumHeight(40);
    statePlayBtn->setStyleSheet("font-size:20px;");
    rightLayout->addWidget(statePlayBtn);
    statePauseBtn = new QPushButton("Pause");
    statePauseBtn->setMinimumHeight(40);
    statePauseBtn->setStyleSheet("font-size:20px;");
    rightLayout->addWidget(statePauseBtn);
    stateFinishBtn = new QPushButton("Finish");
    stateFinishBtn->setMinimumHeight(40);
    stateFinishBtn->setStyleSheet("font-size:20px;");
    rightLayout->addWidget(stateFinishBtn);

    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(rightLayout);
    QWidget *mainWidget = new QWidget();
    mainWidget->setLayout(mainLayout);
    setCentralWidget(mainWidget);

    fTimer = new QTimer();
    connect(fTimer, &QTimer::timeout, this, &CtrlWindow::OnFTimer);
    connect(stateInitBtn, &QPushButton::clicked, this, &CtrlWindow::OnBtnInitClicked);
    connect(stateReadyBtn, &QPushButton::clicked, this, &CtrlWindow::OnBtnReadyClicked);
    connect(statePlayBtn, &QPushButton::clicked, this, &CtrlWindow::OnBtnPlayClicked);
    connect(statePauseBtn, &QPushButton::clicked, this, &CtrlWindow::OnBtnPauseClicked);
    connect(stateFinishBtn, &QPushButton::clicked, this, &CtrlWindow::OnBtnFinishClicked);


    fTimer->start(mBasicTIme);
    remainTime = totalTime;
    paused = true;
    mMs = 0;
    mGdata.mode = mGdata.MODE_NORM;

    mGdata.state = common::msg::GameData::STATE_INIT;
    mGdata.red_players[0].name = "red_1";
    mGdata.red_players[1].name = "red_2";
    mGdata.blue_players[0].name = "blue_1";
    mGdata.blue_players[1].name = "blue_2";
    mGdata.remain_time = totalTime;
    for (size_t i = 0; i < 2; i++) {
        mGdata.red_players[i].state = common::msg::Player::PLAYER_NORMAL;
        mGdata.blue_players[i].state = common::msg::Player::PLAYER_NORMAL;
    }
    gameDataPublisher = std::make_shared<GameDataPublisher>();
    fieldDataSubscriber = std::make_shared<FieldDataSubscriber>();
}

void CtrlWindow::OnFTimer()
{
    std::map<int, QString> infos = {
        {mFdata.BALL_OUT, "ball out of field"},
        {mFdata.BALL_NOMOVE, "ball does not move"},
        {mFdata.BALL_GOAL, "goal"},
    };
    rclcpp::spin_some(gameDataPublisher);
    rclcpp::spin_some(fieldDataSubscriber);
    mFdata = fieldDataSubscriber->GetData();
    if (mFdata.ball_state != mFdata.BALL_NORMAL && mGdata.state == mGdata.STATE_PLAY) {
        mGdata.red_score = mFdata.red_score;
        mGdata.blue_score = mFdata.blue_score;
        mGdata.state = mGdata.STATE_PAUSE;
        OnBtnPauseClicked();
        infoLabel->setText(infos[mFdata.ball_state]);
    }
    blueTeam->SetScore(mGdata.blue_score);
    redTeam->SetScore(mGdata.red_score);
    mGdata.remain_time = remainTime;
    for (size_t i = 0; i < 2; i++) {
        if (mFdata.red_players[i].state == common::msg::Player::PALYER_OUT) {
            mGdata.red_players[i].state = common::msg::Player::PLAYER_WAIT;
            mGdata.red_players[i].wait_time = remainTime;
        }
        if (mFdata.blue_players[i].state == common::msg::Player::PALYER_OUT) {
            mGdata.blue_players[i].state = common::msg::Player::PLAYER_WAIT;
            mGdata.blue_players[i].wait_time = remainTime;
        }
    }
    gameDataPublisher->Publish(mGdata);
    if(paused) {
        return;
    }
    mMs += mBasicTIme;
    if(mMs % 1000 == 0) {
        remainTime--;
        timeLabel->setText(QString::number(remainTime));
        if(remainTime <= 0) {
            OnBtnFinishClicked();
        }
        for (size_t i = 0; i < 2; i++) {
            if (mGdata.red_players[i].state == common::msg::Player::PLAYER_WAIT) {
                if (mGdata.red_players[i].wait_time - remainTime > 30) {
                    mGdata.red_players[i].state = common::msg::Player::PLAYER_NORMAL;
                }
            }
            if (mGdata.blue_players[i].state == common::msg::Player::PLAYER_WAIT) {
                if (mGdata.blue_players[i].wait_time - remainTime > 30) {
                    mGdata.blue_players[i].state = common::msg::Player::PLAYER_NORMAL;
                }
            }
        }
    }
}

void CtrlWindow::OnBtnInitClicked()
{
    paused = true;
    mGdata.state = common::msg::GameData::STATE_INIT;
    stateLabel->setText(QString::fromStdString(states[mGdata.state]));
}

void CtrlWindow::OnBtnReadyClicked()
{
    paused = true;
    mGdata.state = common::msg::GameData::STATE_READY;
    stateLabel->setText(QString::fromStdString(states[mGdata.state]));
}

void CtrlWindow::OnBtnPlayClicked()
{
    paused = false;
    mGdata.state = common::msg::GameData::STATE_PLAY;
    stateLabel->setText(QString::fromStdString(states[mGdata.state]));
}

void CtrlWindow::OnBtnPauseClicked()
{
    paused = true;
    mGdata.state = common::msg::GameData::STATE_PAUSE;
    stateLabel->setText(QString::fromStdString(states[mGdata.state]));
}

void CtrlWindow::OnBtnFinishClicked()
{
    paused = true;
    mGdata.state = common::msg::GameData::STATE_END;
    stateLabel->setText(QString::fromStdString(states[mGdata.state]));
    QString endTime = QTime::currentTime().toString("HH:mm:ss");
    ofstream ofs("results.txt", ios::out | ios::app);
    if(!ofs) {
        return;
    }
    ofs << startTime.toStdString() << " --- " << endTime.toStdString() << endl;
    ofs << redName.toStdString() << " : " << blueName.toStdString() << "\t" << mFdata.red_score << " : " << mFdata.blue_score << endl;
    ofs << endl;
    ofs.close();
}