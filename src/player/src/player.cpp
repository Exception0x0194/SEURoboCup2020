#include "topics.hpp"

enum status
{
    SEARCHING_BALL,
    REACHING_BALL,
    KICKING_BALL
} currentStatus;

const float ballThresholdLeft = 0.375, ballThresholdRight = 0.5;
const float ballThresholdMiddle = (ballThresholdLeft + ballThresholdRight) / 2;
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
    auto binImgPublisher = std::make_shared<ResultImagePublisher>("binary");
    rclcpp::WallRate loop_rate(10.0);
    cv::Vec3f position;

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

            cv::Mat binImage;
            cv::cvtColor(image, binImage, cv::COLOR_BGR2GRAY);
            cv::threshold(binImage, binImage, 230, 255, cv::THRESH_BINARY);
            cv::GaussianBlur(binImage, binImage, cv::Size(5, 5), 3, 3);

            position = (0, 0, 0);
            if (currentStatus == REACHING_BALL)
            {
                std::vector<cv::Vec3f> circles;
                cv::HoughCircles(binImage, circles, cv::HOUGH_GRADIENT, 1, 100, 45, 30, 5, 220);
                if (circles.size())
                {
                    for (size_t i = 0; i < circles.size(); i++)
                    {
                        position[0] += circles[i][0];
                        position[1] += circles[i][1];
                        position[2] += circles[i][2];
                    }
                    position[0] /= circles.size(), position[1] /= circles.size(), position[2] /= circles.size();
                    cv::circle(outputImage, cv::Point(position[0], position[1]), position[2], cv::Scalar(255, 0, 0), 5);
                }
            }
            else if (currentStatus == KICKING_BALL)
            {
                //TODO
            }

            cv::cvtColor(binImage, binImage, cv::COLOR_GRAY2BGR);
            binImgPublisher->Publish(binImage);
            resImgPublisher->Publish(outputImage); // 处理完的图像可以通过该方式发布出去，然后通过rqt中的image_view工具查看
        }
        // Actions
        if (currentStatus == REACHING_BALL && position[2] != 0)
        {
            if (position[0] > ballThresholdRight * image.cols || position[0] < ballThresholdLeft * image.cols)
            {
                btask.set__turn((ballThresholdMiddle * image.cols - position[0]) * turningPerPixel);
                btask.set__step(0);
            }
            else
            {
                btask.set__step(1);
            }
        }

        bodyTaskNode->Publish(btask);
        headTaskNode->Publish(htask);
        loop_rate.sleep();
    }
    return 0;
}
