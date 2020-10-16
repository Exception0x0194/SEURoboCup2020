#include "topics.hpp"

enum status
{
    SEARCHING_BALL,
    REACHING_BALL,
    KICKING_BALL,
    KICKING_ACTION
} currentStatus;

const float ballThresholdLeft = 0.375, ballThresholdRight = 0.5;
const float ballThresholdBottom = 0.8;
const float ballThresholdMiddle = (ballThresholdLeft + ballThresholdRight) / 2;
const float gateThresholdLeft = 0.4375, gateThresholdRight = 0.5625;
const float turningPerPixel = 0.02;

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
    btask.count = 2;
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
    rclcpp::WallRate loop_rate(10.0);
    int cnt = 0;
    cv::Vec3f ballPosition;
    cv::Vec4i linePosition;

    while (rclcpp::ok())
    {
        rclcpp::spin_some(bodyTaskNode);
        rclcpp::spin_some(headTaskNode);
        rclcpp::spin_some(imageSubscriber);
        rclcpp::spin_some(imuSubscriber);
        rclcpp::spin_some(headSubscriber);
        rclcpp::spin_some(resImgPublisher);
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

            cv::Mat binImage;
            cv::cvtColor(image, binImage, cv::COLOR_BGR2GRAY);
            cv::threshold(binImage, binImage, 230, 255, cv::THRESH_BINARY);
            cv::GaussianBlur(binImage, binImage, cv::Size(5, 5), 3, 3);

            ballPosition[0] = ballPosition[1] = ballPosition[2] = 0;
            linePosition[0] = linePosition[1] = linePosition[2] = linePosition[3] = 0;
            if (currentStatus == REACHING_BALL)
            {
                htask.set__pitch(60);
                std::vector<cv::Vec3f> circles;
                cv::HoughCircles(binImage, circles, cv::HOUGH_GRADIENT, 1, 100, 45, 30, 5, 220);
                if (circles.size())
                {
                    for (size_t i = 0; i < circles.size(); i++)
                    {
                        ballPosition[0] += circles[i][0];
                        ballPosition[1] += circles[i][1];
                        ballPosition[2] += circles[i][2];
                    }
                    ballPosition[0] /= circles.size(), ballPosition[1] /= circles.size(), ballPosition[2] /= circles.size();
                    cv::circle(outputImage, cv::Point(ballPosition[0], ballPosition[1]), ballPosition[2], cv::Scalar(255, 0, 0), 5);
                    if (ballPosition[1] > ballThresholdBottom * image.rows)
                    {
                        currentStatus = KICKING_BALL;
                    }
                }
            }
            if (currentStatus == KICKING_BALL)
            {
                htask.set__pitch(0);
                std::vector<cv::Vec4i> lines;
                cv::HoughLinesP(binImage, lines, 1, CV_PI / 180, 100, 50, 30);
                if (lines.size())
                {
                    for (size_t i = 0; i < lines.size(); i++)
                    {
                        linePosition[0] += lines[i][0];
                        linePosition[1] += lines[i][1];
                        linePosition[2] += lines[i][2];
                        linePosition[3] += lines[i][3];
                    }
                    linePosition[1] /= lines.size(), linePosition[1] /= lines.size(), linePosition[2] /= lines.size(), linePosition[3] /= lines.size();
                    cv::line(outputImage, cv::Point(linePosition[0], linePosition[1]), cv::Point(linePosition[2], linePosition[3]), cv::Scalar(0, 255, 255), 5);
                }
            }

            cv::cvtColor(binImage, binImage, cv::COLOR_GRAY2BGR);
            resImgPublisher->Publish(outputImage); // 处理完的图像可以通过该方式发布出去，然后通过rqt中的image_view工具查看
        }
        // Actions
        if (currentStatus == REACHING_BALL && ballPosition[2] != 0)
        {
            if (ballPosition[0] > ballThresholdRight * image.cols || ballPosition[0] < ballThresholdLeft * image.cols)
            {
                btask.set__turn((ballThresholdMiddle * image.cols - ballPosition[0]) * turningPerPixel);
                btask.set__step(0);
            }
            else
            {
                btask.set__turn(0);
                btask.set__step(1);
            }
        }
        else if (currentStatus == KICKING_BALL && linePosition[0] != 0)
        {
            cv::Vec2f gatePosition;
            gatePosition[0] = (linePosition[0] + linePosition[2]) / 2;
            gatePosition[1] = (linePosition[1] + linePosition[3]) / 2;
            if (gatePosition[0] > gateThresholdRight * image.cols || gatePosition[0] < gateThresholdLeft * image.cols)
            {
                btask.set__turn((0.5 * image.cols - gatePosition[0]) * turningPerPixel); // 0.5 is gateThresholdMiddle
                btask.set__lateral(-(0.5 * image.cols - gatePosition[0]) * turningPerPixel);
                btask.set__step(0);
            }
            else
            {
                btask.set__type(btask.TASK_ACT);
                currentStatus = KICKING_ACTION;
                cnt = 0;
            }
        }
        else if (currentStatus == KICKING_ACTION)
        {
            cnt++;
            if (cnt == 10)
            {
                currentStatus = REACHING_BALL;
                btask.set__type(btask.TASK_WALK);
            }
        }

        bodyTaskNode->Publish(btask);
        headTaskNode->Publish(htask);
        loop_rate.sleep();
    }
    return 0;
}
