#include "topics.hpp"

enum status
{
    SEARCHING_BALL,
    REACHING_BALL,
    KICKING_BALL,
    KICKING_ACTION
} currentStatus;

const float ballThresholdLeft = 0.375, ballThresholdRight = 0.47;
// const float ballThresholdLeft = 0.4375, ballThresholdRight = 0.5625;
const float ballThresholdBottom = 0.8;
const float ballThresholdMiddle = (ballThresholdLeft + ballThresholdRight) / 2;
const float gateThresholdLeft = 0.43, gateThresholdRight = 0.57;
const float turningPerPixel = 0.04, lateralMovingPerPixel = -0.000012;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string robotName = "maxwell";
    if (argc > 1)
    {
        robotName = std::string(argv[1]);
    }
    auto playerNode = std::make_shared<rclcpp::Node>(robotName + "_player");
    common::msg::BodyTask btask;
    common::msg::HeadTask htask;
    btask.type = btask.TASK_WALK;
    btask.count = 1;
    btask.step = 0.03;
    htask.yaw = 0.0;
    htask.pitch = 45.0;
    currentStatus = REACHING_BALL;
    auto bodyTaskNode = std::make_shared<BodyTaskPublisher>(robotName);
    auto headTaskNode = std::make_shared<HeadTaskPublisher>(robotName);
    auto imageSubscriber = std::make_shared<ImageSubscriber>(robotName);
    auto imuSubscriber = std::make_shared<ImuDataSubscriber>(robotName);
    auto headSubscriber = std::make_shared<HeadAngleSubscriber>(robotName);
    auto resImgPublisher = std::make_shared<ResultImagePublisher>(robotName);
    auto binImgPublisher = std::make_shared<ResultImagePublisher>("bin");
    rclcpp::WallRate loop_rate(10.0);
    int cnt = 0;
    cv::Vec3f ballPosition;
    cv::Vec4i gatePosition;

    while (rclcpp::ok())
    {
        rclcpp::spin_some(bodyTaskNode);
        rclcpp::spin_some(headTaskNode);
        rclcpp::spin_some(imageSubscriber);
        rclcpp::spin_some(imuSubscriber);
        rclcpp::spin_some(headSubscriber);
        rclcpp::spin_some(resImgPublisher);
        rclcpp::spin_some(binImgPublisher);
        auto imuData = imuSubscriber->GetData();
        auto image = imageSubscriber->GetImage().clone();
        auto headAngle = headSubscriber->GetData();

        if (!image.empty())
        {
            // Image processing
            auto image = imageSubscriber->GetImage().clone();
            auto outputImage = image.clone();
            auto imuData = imuSubscriber->GetData();
            auto headAngle = headSubscriber->GetData();
            auto grayImage = image.clone();
            cv::cvtColor(grayImage, grayImage, cv::COLOR_BGR2GRAY);
            ballPosition[0] = ballPosition[1] = ballPosition[2] = 0;
            gatePosition[0] = gatePosition[1] = gatePosition[2] = gatePosition[3] = 0;

            if (currentStatus == REACHING_BALL)
            {
                auto whiteBinImage = grayImage.clone(), blackBinImage = grayImage.clone(), ballBinImage = grayImage.clone();
                cv::threshold(whiteBinImage, whiteBinImage, 220, 255, cv::THRESH_BINARY);
                cv::threshold(blackBinImage, blackBinImage, 30, 255, cv::THRESH_BINARY_INV);
                cv::bitwise_or(whiteBinImage, blackBinImage, ballBinImage);
                cv::GaussianBlur(ballBinImage, ballBinImage, cv::Size(7, 7), 1, 1);
                //cv::medianBlur(ballBinImage, ballBinImage, 5);
                cv::Canny(ballBinImage, ballBinImage, 75, 225);

                cv::cvtColor(ballBinImage, outputImage, cv::COLOR_GRAY2BGR);

                std::vector<cv::Vec3f> circles;
                cv::HoughCircles(ballBinImage, circles, cv::HOUGH_GRADIENT, 1, 100, 45, 30, 10, 220);
                if (circles.size())
                {
                    for (int i = 0; i < circles.size(); i++)
                    {
                        if (ballPosition[2] == 0 || circles[i][1] > ballPosition[1])
                            ballPosition = circles[i];
                        cv::circle(outputImage, cv::Point(circles[i][0], circles[i][1]), circles[i][2], cv::Scalar(0, 255, 255), 5);
                    }
                    cv::circle(outputImage, cv::Point(ballPosition[0], ballPosition[1]), ballPosition[2], cv::Scalar(255, 0, 0), 5);
                }
                else
                {
                    //TODO
                }
            }
            if (1)
            {
                auto whiteBinImage = grayImage.clone(), blackBinImage = grayImage.clone(), gateBinImage = grayImage.clone();
                cv::threshold(whiteBinImage, whiteBinImage, 190, 255, cv::THRESH_BINARY_INV);
                cv::threshold(blackBinImage, blackBinImage, 180, 255, cv::THRESH_BINARY);
                cv::bitwise_and(whiteBinImage, blackBinImage, gateBinImage);
                cv::GaussianBlur(gateBinImage, gateBinImage, cv::Size(3, 3), 3, 3);
                cv::threshold(gateBinImage, gateBinImage, 120, 255, cv::THRESH_BINARY);

                std::vector<cv::Vec4i> lines;
                int lineNumber = 0;
                cv::HoughLinesP(gateBinImage, lines, 1, CV_PI / 180, 50, 50, 10);
                if (lines.size())
                {
                    for (int i = 0; i < lines.size(); i++)
                    {
                        if (lines[i][0] != lines[i][2] && (abs(float(lines[i][1] - lines[i][3]) / (lines[i][0] - lines[i][2])) < 0.5))
                        {
                            gatePosition += lines[i];
                            lineNumber++;
                        }
                    }
                    gatePosition /= (lineNumber ? lineNumber : 1);
                }
            }
            else
            {
                //TODO
            }

            // cv::cvtColor(binImage, binImage, cv::COLOR_GRAY2BGR);
            // if (robotName.back() == '1')
            //     binImgPublisher->Publish(binImage);
            resImgPublisher->Publish(outputImage); // 处理完的图像可以通过该方式发布出去，然后通过rqt中的image_view工具查看
        }

        // Actions
        if (currentStatus == REACHING_BALL && ballPosition[2] != 0)
        {
            htask.pitch = 40;
            btask.type = btask.TASK_WALK;
            if (ballPosition[0] < ballThresholdLeft * image.cols || ballPosition[0] > ballThresholdRight * image.cols)
            {
                btask.turn = ((ballThresholdMiddle * image.cols - ballPosition[0]) * turningPerPixel);
                btask.step = 0;
            }
            else
            {
                if (ballPosition[1] < ballThresholdBottom * image.rows)
                {
                    btask.turn = 0;
                    btask.step = (ballPosition[1] < 0.5 * image.rows ? 1 : 0.8);
                }
                else
                {
                    btask.turn = 0;
                    btask.step = 0;
                    htask.pitch = 0;
                    currentStatus = KICKING_BALL;
                    cnt = 0;
                }
            }
        }
        else if (currentStatus == KICKING_BALL && (gatePosition[0] != gatePosition[2] && gatePosition[1] != gatePosition[3]))
        {
            htask.pitch = 0;
            btask.turn = 0;
            btask.step = 0;
            int gatePositionAvg = (gatePosition[0] + gatePosition[2]) / 2;
            if (gatePositionAvg < gateThresholdLeft * image.cols || gatePositionAvg > gateThresholdRight * image.cols)
            {
                btask.turn = ((0.5 * image.cols - gatePositionAvg) * turningPerPixel);
                btask.lateral = ((0.5 * image.cols - gatePositionAvg) * lateralMovingPerPixel);
            }
            else if (1)
            {
                btask.step = 0;
                btask.turn = 0;
                btask.lateral = 0;
                btask.type = btask.TASK_ACT;
                btask.actname = "left_kick";
                cnt = 0;
                currentStatus = KICKING_ACTION;
            }
        }
        else if (currentStatus == KICKING_ACTION)
        {
            cnt++;
            if (cnt >= 5)
            {
                currentStatus = REACHING_BALL;
                btask.type = btask.TASK_WALK;
                htask.pitch = 40;
                btask.step = 0;
                btask.turn = 0;
                btask.lateral = 0;
            }
        }
        else
        {
            btask.type = btask.TASK_WALK;
            btask.step = 1;
            btask.turn = 0;
            btask.lateral = 0;
        }

        bodyTaskNode->Publish(btask);
        headTaskNode->Publish(htask);
        loop_rate.sleep();
    }
    return 0;
}
