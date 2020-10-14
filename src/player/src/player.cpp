#include "topics.hpp"

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
    auto bodyTaskNode = std::make_shared<BodyTaskPublisher>(robotName);
    auto headTaskNode = std::make_shared<HeadTaskPublisher>(robotName);
    auto imageSubscriber = std::make_shared<ImageSubscriber>(robotName);
    auto imuSubscriber = std::make_shared<ImuDataSubscriber>(robotName);
    auto headSubscriber = std::make_shared<HeadAngleSubscriber>(robotName);
    auto resImgPublisher = std::make_shared<ResultImagePublisher>(robotName);
    // auto resImgPublisher2 = std::make_shared<ResultImagePublisher>("Img");
    rclcpp::WallRate loop_rate(10.0);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(bodyTaskNode);
        rclcpp::spin_some(headTaskNode);
        rclcpp::spin_some(imageSubscriber);
        rclcpp::spin_some(imuSubscriber);
        rclcpp::spin_some(headSubscriber);
        rclcpp::spin_some(resImgPublisher);
        // rclcpp::spin_some(resImgPublisher2);
        auto imuData = imuSubscriber->GetData();
        auto image = imageSubscriber->GetImage().clone();
        auto headAngle = headSubscriber->GetData();

        if (!image.empty())
        {
            // 在这里写图像处理
            //cv::circle(image, cv::Point(0, 0), 40, cv::Scalar(255, 0, 0));
            auto image = imageSubscriber->GetImage().clone();
            auto imuData = imuSubscriber->GetData();
            auto headAngle = headSubscriber->GetData();
            cv::Mat grayImage, binImage, outputImage, kernel, dstImage;
            cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
            cv::threshold(grayImage, binImage, 230, 255, cv::THRESH_BINARY);
            cv::GaussianBlur(binImage, binImage, cv::Size(5, 5), 3, 3);

            std::vector<cv::Vec3f> circles;
            cv::HoughCircles(binImage, circles, cv::HOUGH_GRADIENT, 1, 100, 45, 30, 10, 220);
            cv::cvtColor(binImage, outputImage, cv::COLOR_GRAY2BGR);
            for (size_t i = 0; i < circles.size(); i++)
            {
                cv::Vec3f c = circles[i];
                cv::circle(outputImage, cv::Point(c[0], c[1]), c[2], cv::Scalar(0, 255, 255), 5);
            }
            cv::circle(outputImage, cv::Point(0, 0), 50, cv::Scalar(0, 255, 255), 5);
            resImgPublisher->Publish(outputImage); // 处理完的图像可以通过该方式发布出去，然后通过rqt中的image_view工具查看
        }

        if (robotName.back() == '1')
        {
            btask.step = 2; // 1 号机器人前进
        }

        bodyTaskNode->Publish(btask);
        headTaskNode->Publish(htask);
        loop_rate.sleep();
    }
    return 0;
}
