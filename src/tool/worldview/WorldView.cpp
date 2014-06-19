#include "WorldView.h"
#include <iostream>
#include <string>
#include <QIntValidator>
#include <QDebug>

namespace tool {
namespace worldview {

WorldView::WorldView(QWidget* parent)
    : portals::Module(),
      QWidget(parent),
      commThread("comm", COMM_FRAME_LENGTH_uS),
      wviewComm(16,0),
      newTeam(0),
      mutex()
{
    commThread.addModule(*this);
    commThread.addModule(wviewComm);
    for (int i = 0; i < NUM_PLAYERS_PER_TEAM; i++) {
        commThread.addModule(wviewTeammate[i]);
    }

#ifdef USING_LAB_FIELD
    fieldPainter = new WorldViewPainter(this, 2.);
#else
    fieldPainter = new WorldViewPainter(this, 1.);
#endif

    QHBoxLayout *mainLayout = new QHBoxLayout(this);

    QHBoxLayout *field = new QHBoxLayout();
    field->addWidget(fieldPainter);

    QVBoxLayout *rightBar = new QVBoxLayout();

    QVBoxLayout *options = new QVBoxLayout();
    options->setAlignment(Qt::AlignTop);
    startButton = new QPushButton(QString("Start World Viewer"));
    options->addWidget(startButton);

    QHBoxLayout *teamLayout = new QHBoxLayout();
    QLabel *teamLabel = new QLabel(tr("Listening to Team: "));
    teamSelector = new QLineEdit(tr("16"));
    QValidator *teamVal = new QIntValidator(1, 255);
    teamSelector->setValidator(teamVal);
    teamLayout->addWidget(teamLabel);
    teamLayout->addWidget(teamSelector);
    options->addLayout(teamLayout);

    connect(teamSelector, SIGNAL(editingFinished()), this, SLOT(teamChanged()));

    QVBoxLayout *stateLayout = new QVBoxLayout();
    stateLayout->setAlignment(Qt::AlignBottom);

    QGroupBox *stateBox = new QGroupBox(tr("Robot States"));
    QVBoxLayout *boxLayout = new QVBoxLayout();
    for (int i = 0; i < NUM_PLAYERS_PER_TEAM; ++i)
    {
        roleLabels[i] = new QLabel(tr("Inactive"));
    }
    QHBoxLayout *p1Layout = new QHBoxLayout();
    QLabel *p1Label = new QLabel(tr("Player 1: "));
    QHBoxLayout *p2Layout = new QHBoxLayout();
    QLabel *p2Label = new QLabel(tr("Player 2: "));
    QHBoxLayout *p3Layout = new QHBoxLayout();
    QLabel *p3Label = new QLabel(tr("Player 3: "));
    QHBoxLayout *p4Layout = new QHBoxLayout();
    QLabel *p4Label = new QLabel(tr("Player 4: "));
    QHBoxLayout *p5Layout = new QHBoxLayout();
    QLabel *p5Label = new QLabel(tr("Player 5: "));


    for (int i = 0; i < NUM_PLAYERS_PER_TEAM; ++i)
    {
        rolePrediction[i] = new QLabel(tr("Unknown"));
        robotReliability[i] = new QLabel(tr("NA"));
    }
    QHBoxLayout *testP1LayoutG = new QHBoxLayout();
    QLabel *testP1LabelG = new QLabel(tr("P1 Guess: "));
    QHBoxLayout *testP1LayoutR = new QHBoxLayout();
    QLabel *testP1LabelR = new QLabel(tr("P1 Reliability: "));
    QHBoxLayout *testP2LayoutG = new QHBoxLayout();
    QLabel *testP2LabelG = new QLabel(tr("P2 Guess: "));
    QHBoxLayout *testP2LayoutR = new QHBoxLayout();
    QLabel *testP2LabelR = new QLabel(tr("P2 Reliability: "));
    QHBoxLayout *testP3LayoutG = new QHBoxLayout();
    QLabel *testP3LabelG = new QLabel(tr("P3 Guess: "));
    QHBoxLayout *testP3LayoutR = new QHBoxLayout();
    QLabel *testP3LabelR = new QLabel(tr("P3 Reliability: "));
    QHBoxLayout *testP4LayoutG = new QHBoxLayout();
    QLabel *testP4LabelG = new QLabel(tr("P4 Guess:  "));
    QHBoxLayout *testP4LayoutR = new QHBoxLayout();
    QLabel *testP4LabelR = new QLabel(tr("P4 Reliability: "));
    QHBoxLayout *testP5LayoutG = new QHBoxLayout();
    QLabel *testP5LabelG = new QLabel(tr("P5 Guess: "));
    QHBoxLayout *testP5LayoutR = new QHBoxLayout();
    QLabel *testP5LabelR = new QLabel(tr("P5 Reliability: "));

    QHBoxLayout *separator1 = new QHBoxLayout();
    QLabel *separatorLabel1 = new QLabel(tr(" "));
    QHBoxLayout *separator2 = new QHBoxLayout();
    QLabel *separatorLabel2 = new QLabel(tr(" "));

    p1Layout->addWidget(p1Label);
    p1Layout->addWidget(roleLabels[0]);

    p2Layout->addWidget(p2Label);
    p2Layout->addWidget(roleLabels[1]);

    p3Layout->addWidget(p3Label);
    p3Layout->addWidget(roleLabels[2]);

    p4Layout->addWidget(p4Label);
    p4Layout->addWidget(roleLabels[3]);

    p5Layout->addWidget(p5Label);
    p5Layout->addWidget(roleLabels[4]);

    separator1->addWidget(separatorLabel1);
    separator2->addWidget(separatorLabel2);

    // this is for testing the teammate interpreter module
    testP1LayoutG->addWidget(testP1LabelG);
    testP1LayoutG->addWidget(rolePrediction[0]);
    testP1LayoutR->addWidget(testP1LabelR);
    testP1LayoutR->addWidget(robotReliability[0]);
    testP2LayoutG->addWidget(testP2LabelG);
    testP2LayoutG->addWidget(rolePrediction[1]);
    testP2LayoutR->addWidget(testP2LabelR);
    testP2LayoutR->addWidget(robotReliability[1]);
    testP3LayoutG->addWidget(testP3LabelG);
    testP3LayoutG->addWidget(rolePrediction[2]);
    testP3LayoutR->addWidget(testP3LabelR);
    testP3LayoutR->addWidget(robotReliability[2]);
    testP4LayoutG->addWidget(testP4LabelG);
    testP4LayoutG->addWidget(rolePrediction[3]);
    testP4LayoutR->addWidget(testP4LabelR);
    testP4LayoutR->addWidget(robotReliability[3]);
    testP5LayoutG->addWidget(testP5LabelG);
    testP5LayoutG->addWidget(rolePrediction[4]);
    testP5LayoutR->addWidget(testP5LabelR);
    testP5LayoutR->addWidget(robotReliability[4]);

    boxLayout->addLayout(p1Layout);
    boxLayout->addLayout(p2Layout);
    boxLayout->addLayout(p3Layout);
    boxLayout->addLayout(p4Layout);
    boxLayout->addLayout(p5Layout);

    boxLayout->addLayout(separator1);

    boxLayout->addLayout(testP1LayoutG);
    boxLayout->addLayout(testP2LayoutG);
    boxLayout->addLayout(testP3LayoutG);
    boxLayout->addLayout(testP4LayoutG);
    boxLayout->addLayout(testP5LayoutG);

    boxLayout->addLayout(separator2);

    boxLayout->addLayout(testP1LayoutR);
    boxLayout->addLayout(testP2LayoutR);
    boxLayout->addLayout(testP3LayoutR);
    boxLayout->addLayout(testP4LayoutR);
    boxLayout->addLayout(testP5LayoutR);

    boxLayout->setSpacing(10);

    stateBox->setFlat(false);

    stateBox->setLayout(boxLayout);
    stateLayout->addWidget(stateBox);

    rightBar->addLayout(options);
    rightBar->addLayout(stateLayout);

    mainLayout->addLayout(field);
    mainLayout->addLayout(rightBar);

    this->setLayout(mainLayout);

    connect(startButton, SIGNAL(clicked()), this, SLOT(startButtonClicked()));

    for (int i = 0; i < NUM_PLAYERS_PER_TEAM; ++i)
    {
        commIn[i].wireTo(wviewComm._worldModels[i]);
        wviewTeammate[i].worldModelIn.wireTo(wviewComm._worldModels[i]);
        teammateIn[i].wireTo(&wviewTeammate[i].teammateInterpreterOutput);
    }
}


void WorldView::run_()
{
    mutex.lock();
    if (newTeam)
    {
        wviewComm.setTeamNumber(newTeam);
        newTeam = 0;

        qDebug() << "World View now listening to team: " << wviewComm.teamNumber();
    }

    for (int i = 0; i < NUM_PLAYERS_PER_TEAM; ++i)
    {
        commIn[i].latch();
        fieldPainter->updateWithLocationMessage(commIn[i].message(), i);
        updateStatus(commIn[i].message(), i);

        teammateIn[i].latch();
        updateRoleGuess(teammateIn[i].message(), i);
    }
    mutex.unlock();
}

void WorldView::startButtonClicked()
{
    mutex.lock();
    commThread.start();
    startButton->setText(QString("Stop World Viewer"));
    disconnect(startButton, SIGNAL(clicked()), this, SLOT(startButtonClicked()));
    connect(startButton, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));
    mutex.unlock();
}

void WorldView::stopButtonClicked()
{
    mutex.lock();
    commThread.stop();
    startButton->setText(QString("Start World Viewer"));
    disconnect(startButton, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));
    connect(startButton, SIGNAL(clicked()), this, SLOT(startButtonClicked()));
    mutex.unlock();
}

void WorldView::teamChanged()
{
    mutex.lock();
    newTeam = teamSelector->text().toInt();
    mutex.unlock();
}

void WorldView::updateStatus(messages::WorldModel msg, int index)
{
    if (!msg.active()) {
        roleLabels[index]->setText(QString("Inactive"));
    } else {
        roleLabels[index]->setText(roles[msg.role() - 1]);
    }
}

void WorldView::updateRoleGuess(messages::TeammateInterpreter msg, int index)
{
    if (!msg.player_role()) {
        rolePrediction[index]->setText(QString("Unknown"));
    } else {
        rolePrediction[index]->setText(roles[msg.player_role() - 1]);
    }

    if (msg.reliability() == -1) {
        robotReliability[index]->setText(QString("NA"));
    } else {
        robotReliability[index]->setText(QString::number(msg.reliability()));
    }
}

} // namespace worldview
} // namespace man
