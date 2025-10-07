#include <chrono>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <cmath> // For std::fabs

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "hik_camera_ros2/MvCameraControl.h"

// 将海康相机像素格式转换为ROS图像编码格式
std::string MvPixelTypeToString(MvGvspPixelType enType)
{
    switch (enType)
    {
    case PixelType_Gvsp_Mono8:
        return "mono8";
    case PixelType_Gvsp_BGR8_Packed:
        return "bgr8";
    case PixelType_Gvsp_RGB8_Packed:
        return "rgb8";
    case PixelType_Gvsp_BayerGR8:
        return "bayer_grbg8";
    case PixelType_Gvsp_BayerRG8:
        return "bayer_rggb8";
    case PixelType_Gvsp_BayerGB8:
        return "bayer_gbrg8";
    case PixelType_Gvsp_BayerBG8:
        return "bayer_bggr8";
    default:
        return "unknown";
    }
}

class HikRos2CameraNode : public rclcpp::Node
{
public:
    explicit HikRos2CameraNode(const rclcpp::NodeOptions & options) 
    : Node("hik_camera_node", options), 
      handle_(nullptr), 
      is_connected_(false)
    {
        // ==================== 1. ROS2参数声明和初始化 ====================
        declare_parameters();
        
        // ==================== 2. 创建图像发布者 ====================
        std::string topic_name = this->get_parameter("topic_name").as_string();
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);

        // ==================== 3. [修改] 初始化SDK，整个生命周期只执行一次 ====================
        int nRet = MV_CC_Initialize();
        if (MV_OK != nRet) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize SDK! nRet [0x%x]", nRet);
            // 如果SDK初始化失败，节点无法工作，直接关闭
            if (rclcpp::ok()) {
                rclcpp::shutdown();
            }
            return;
        }

        // ==================== 4. 设置参数回调 ====================
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&HikRos2CameraNode::on_parameter_set, this, std::placeholders::_1));

        // ==================== 5. 创建定时器，用于抓取、发布图像和处理重连 ====================
        grab_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // [修改] 降低轮询频率，10Hz足够用于重连检测和图像抓取
            std::bind(&HikRos2CameraNode::grab_publish_and_reconnect, this));
        
        // ==================== 6. 创建定时器，用于从相机同步参数状态 ====================
        param_sync_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), // 每秒同步一次
            std::bind(&HikRos2CameraNode::sync_parameters_from_camera, this));

        RCLCPP_INFO(this->get_logger(), "Hikvision camera node has been initialized. Attempting first connection...");
    }

    ~HikRos2CameraNode()
    {
        if (handle_ != nullptr)
        {
            MV_CC_StopGrabbing(handle_);
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
        }
        // [修改] SDK反初始化，同样只执行一次
        MV_CC_Finalize();
        RCLCPP_INFO(this->get_logger(), "Hikvision camera node has been destroyed.");
    }

private:
    // ==================== ROS2参数相关函数 ====================
    void declare_parameters()
    {
        this->declare_parameter<std::string>("camera_sn", "");
        this->declare_parameter<double>("exposure_time", 5000.0);
        this->declare_parameter<double>("gain", 5.0);
        this->declare_parameter<double>("frame_rate", 30.0);
        this->declare_parameter<std::string>("pixel_format", "bgr8");
        this->declare_parameter<std::string>("camera_frame_id", "camera_frame");
        this->declare_parameter<std::string>("topic_name", "image_raw");

        pixel_format_map_["mono8"] = PixelType_Gvsp_Mono8;
        pixel_format_map_["bgr8"] = PixelType_Gvsp_BGR8_Packed;
        pixel_format_map_["rgb8"] = PixelType_Gvsp_RGB8_Packed;
        pixel_format_map_["bayer_grbg8"] = PixelType_Gvsp_BayerGR8;
        pixel_format_map_["bayer_rggb8"] = PixelType_Gvsp_BayerRG8;
        pixel_format_map_["bayer_gbrg8"] = PixelType_Gvsp_BayerGB8;
        pixel_format_map_["bayer_bggr8"] = PixelType_Gvsp_BayerBG8;
    }

    // ==================== 修改点在这里 ====================
    rcl_interfaces::msg::SetParametersResult on_parameter_set(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        if (!is_connected_ || handle_ == nullptr) {
            result.successful = false;
            result.reason = "Camera is not connected.";
            return result;
        }

        for (const auto &param : parameters)
        {
            std::string name = param.get_name();
            if (name == "exposure_time")
            {
                if (MV_OK != MV_CC_SetFloatValue(handle_, "ExposureTime", param.as_double())) {
                    result.successful = false;
                    result.reason = "Failed to set exposure time.";
                }
            }
            else if (name == "gain")
            {
                if (MV_OK != MV_CC_SetFloatValue(handle_, "Gain", param.as_double())) {
                    result.successful = false;
                    result.reason = "Failed to set gain.";
                }
            }
            else if (name == "frame_rate")
            {
                if (MV_OK != MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", param.as_double())) {
                    result.successful = false;
                    result.reason = "Failed to set frame rate.";
                }
            }
            else if (name == "pixel_format")
            {
                const auto& format_str = param.as_string();
                if (pixel_format_map_.count(format_str)) {
                    // 关键步骤：更改像素格式需要停止/启动流
                    grab_timer_->cancel(); // 暂停抓图定时器，防止冲突
                    
                    int nRet = MV_CC_StopGrabbing(handle_);
                    if (MV_OK != nRet) {
                        result.successful = false;
                        result.reason = "Failed to stop grabbing to change pixel format.";
                        RCLCPP_ERROR(this->get_logger(), "%s Error: [0x%x]", result.reason.c_str(), nRet);
                        // 即使失败，也要尝试恢复
                        grab_timer_->reset();
                        continue; // 处理下一个参数
                    }

                    nRet = MV_CC_SetEnumValue(handle_, "PixelFormat", pixel_format_map_.at(format_str));
                    if (MV_OK != nRet) {
                        result.successful = false;
                        result.reason = "Failed to set PixelFormat value.";
                        RCLCPP_ERROR(this->get_logger(), "%s Error: [0x%x]", result.reason.c_str(), nRet);
                        // 失败后，尝试重启流以恢复相机
                        MV_CC_StartGrabbing(handle_); 
                        grab_timer_->reset();
                        continue;
                    }

                    nRet = MV_CC_StartGrabbing(handle_);
                    if (MV_OK != nRet) {
                        result.successful = false;
                        result.reason = "CRITICAL: Failed to restart grabbing after setting pixel format.";
                        is_connected_ = false; // 标记为断开，让主循环去处理重连
                        RCLCPP_FATAL(this->get_logger(), "%s Error: [0x%x]", result.reason.c_str(), nRet);
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Successfully set PixelFormat to %s", format_str.c_str());
                    }

                    grab_timer_->reset(); // 恢复抓图定时器
                } else {
                    result.successful = false;
                    result.reason = "Unsupported pixel format: " + param.as_string();
                }
            }
            if (!result.successful) {
                // 如果不是pixel_format相关的错误，在这里打印
                if (name != "pixel_format") {
                    RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
                }
            }
        }
        return result;
    }
    // ====================================================

    void sync_parameters_from_camera()
    {
        if (!is_connected_ || handle_ == nullptr) {
            return;
        }

        if (MV_OK != MV_CC_InvalidateNodes(handle_)) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Could not invalidate node cache. Parameter sync may be inaccurate.");
            return;
        }

        std::vector<rclcpp::Parameter> updated_params;
        MVCC_FLOATVALUE stFloatValue{};
        
        if (MV_OK == MV_CC_GetFloatValue(handle_, "ExposureTime", &stFloatValue)) {
            if (std::fabs(this->get_parameter("exposure_time").as_double() - stFloatValue.fCurValue) > 1e-6) {
                updated_params.emplace_back("exposure_time", stFloatValue.fCurValue);
            }
        }

        if (MV_OK == MV_CC_GetFloatValue(handle_, "Gain", &stFloatValue)) {
            if (std::fabs(this->get_parameter("gain").as_double() - stFloatValue.fCurValue) > 1e-6) {
                updated_params.emplace_back("gain", stFloatValue.fCurValue);
            }
        }

        if (MV_OK == MV_CC_GetFloatValue(handle_, "ResultingFrameRate", &stFloatValue)) {
            if (std::fabs(this->get_parameter("frame_rate").as_double() - stFloatValue.fCurValue) > 0.1) {
                updated_params.emplace_back("frame_rate", stFloatValue.fCurValue);
            }
        }
        
        MVCC_ENUMVALUE stEnumValue{};
        if (MV_OK == MV_CC_GetEnumValue(handle_, "PixelFormat", &stEnumValue)) {
            std::string hw_format_str = MvPixelTypeToString(static_cast<MvGvspPixelType>(stEnumValue.nCurValue));
            if (hw_format_str != "unknown" && this->get_parameter("pixel_format").as_string() != hw_format_str) {
                updated_params.emplace_back("pixel_format", hw_format_str);
            }
        }
        
        if (!updated_params.empty()) {
            this->set_parameters(updated_params);
            RCLCPP_INFO(this->get_logger(), "Synchronized parameters from camera hardware.");
        }
    }

    // ==================== 相机控制函数 ====================
    // [修改] connect_camera不再负责SDK初始化
    void connect_camera()
    {
        int nRet = MV_OK;

        memset(&stDeviceList_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList_);
        if (MV_OK != nRet) {
            // 枚举失败，简单返回，等待下次尝试
            return;
        }

        if (stDeviceList_.nDeviceNum == 0) {
            // [修改] 未找到设备是预期行为，安静地返回，等待下次尝试
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Found %d devices. Attempting to establish connection.", stDeviceList_.nDeviceNum);

        std::string camera_sn = this->get_parameter("camera_sn").as_string();
        int device_index = -1;
        if (!camera_sn.empty()) {
            for (unsigned int i = 0; i < stDeviceList_.nDeviceNum; i++) {
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList_.pDeviceInfo[i];
                const char* sn = (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) ? 
                                 (const char*)pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber : 
                                 (const char*)pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber;
                if (camera_sn == std::string(sn)) {
                    device_index = i;
                    RCLCPP_INFO(this->get_logger(), "Found matching camera with SN: %s", camera_sn.c_str());
                    break;
                }
            }
            if (device_index == -1) {
                RCLCPP_WARN(this->get_logger(), "Found devices, but camera with SN [%s] not among them. Will retry.", camera_sn.c_str());
                return;
            }
        } else {
            device_index = 0;
            RCLCPP_INFO(this->get_logger(), "No camera_sn specified, connecting to the first device found.");
        }

        nRet = MV_CC_CreateHandle(&handle_, stDeviceList_.pDeviceInfo[device_index]);
        if (MV_OK != nRet) {
            RCLCPP_ERROR(this->get_logger(), "CreateHandle fail! nRet [0x%x]", nRet);
            return;
        }

        nRet = MV_CC_OpenDevice(handle_);
        if (MV_OK != nRet) {
            RCLCPP_ERROR(this->get_logger(), "OpenDevice fail! nRet [0x%x]", nRet);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Device opened successfully.");
        
        set_initial_parameters();

        nRet = MV_CC_StartGrabbing(handle_);
        if (MV_OK != nRet) {
            RCLCPP_ERROR(this->get_logger(), "StartGrabbing fail! nRet [0x%x]", nRet);
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
            return;
        }

        // [修改] 只有在所有步骤成功后，才将状态设置为已连接
        is_connected_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera connected and started grabbing.");
    }
    
    void set_initial_parameters()
    {
        if(handle_ == nullptr) return;
        MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
        MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
        MV_CC_SetFloatValue(handle_, "ExposureTime", this->get_parameter("exposure_time").as_double());
        MV_CC_SetFloatValue(handle_, "Gain", this->get_parameter("gain").as_double());
        MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", this->get_parameter("frame_rate").as_double());
        
        std::string format_str = this->get_parameter("pixel_format").as_string();
        if (pixel_format_map_.count(format_str)) {
            MV_CC_SetEnumValue(handle_, "PixelFormat", pixel_format_map_.at(format_str));
        } else {
            RCLCPP_WARN(this->get_logger(), "Initial pixel format %s not supported, using camera default.", format_str.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "Initial camera parameters set.");
    }

    // [修改] 重命名并重构了主要逻辑函数
    void grab_publish_and_reconnect()
    {
        // 状态一：已连接，检查是否断线并抓图
        if (is_connected_)
        {
            // 使用IsDeviceConnected检查连接状态
            if (handle_ != nullptr && !MV_CC_IsDeviceConnected(handle_))
            {
                RCLCPP_ERROR(this->get_logger(), "Camera disconnected! Cleaning up resources.");
                // 执行清理，但不立即重连
                MV_CC_StopGrabbing(handle_);
                MV_CC_DestroyHandle(handle_);
                handle_ = nullptr;
                is_connected_ = false;
                // 直接返回，下一次定时器调用将进入“未连接”状态的处理逻辑
                return;
            }

            // 如果仍然连接，则抓取并发布图像
            MV_FRAME_OUT stImageInfo{};
            int nRet = MV_CC_GetImageBuffer(handle_, &stImageInfo, 50); // 使用较短的超时

            if (nRet == MV_OK) {
                auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
                
                image_msg->header.stamp = this->get_clock()->now();
                image_msg->header.frame_id = this->get_parameter("camera_frame_id").as_string();
                image_msg->height = stImageInfo.stFrameInfo.nHeight;
                image_msg->width = stImageInfo.stFrameInfo.nWidth;
                image_msg->encoding = MvPixelTypeToString(stImageInfo.stFrameInfo.enPixelType);
                image_msg->step = stImageInfo.stFrameInfo.nFrameLen / stImageInfo.stFrameInfo.nHeight;
                image_msg->data.resize(stImageInfo.stFrameInfo.nFrameLen);
                memcpy(&image_msg->data[0], stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nFrameLen);

                image_pub_->publish(std::move(image_msg));

                MV_CC_FreeImageBuffer(handle_, &stImageInfo);
            } else {
                // 超时是正常现象，特别是在低帧率下，不打印警告
                if (nRet != static_cast<int>(MV_E_GC_TIMEOUT)) {
                    RCLCPP_WARN(this->get_logger(), "GetImageBuffer fail! nRet [0x%x]", nRet);
                }
            }
        }
        // 状态二：未连接，尝试重连
        else
        {
            // [修改] 使用THROTTLE来避免日志刷屏，每2秒打印一次
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Camera not connected. Attempting to reconnect...");
            connect_camera(); // 尝试连接，此函数成功后会把 is_connected_ 设为 true
        }
    }

    // ==================== 成员变量 ====================
    void *handle_;
    bool is_connected_;
    MV_CC_DEVICE_INFO_LIST stDeviceList_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr grab_timer_;
    rclcpp::TimerBase::SharedPtr param_sync_timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    std::map<std::string, MvGvspPixelType> pixel_format_map_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<HikRos2CameraNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}