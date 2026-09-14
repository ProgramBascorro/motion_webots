/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*
 * robotis_controller.cpp
 *
 *  Created on: 2016. 1. 15.
 *      Author: zerom
 */

#include <yaml-cpp/yaml.h>
#include "robotis_controller/robotis_controller.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace robotis_framework;

RobotisController::RobotisController()
  : Node("robotis_controller"),
    is_timer_running_(false),
    is_offset_enabled_(true),
    offset_ratio_(1.0),
    stop_timer_(false),
    init_pose_loaded_(false),
    timer_thread_(0),
    controller_mode_(MotionModuleMode),
    DEBUG_PRINT(false),
    robot_(0),
    gazebo_mode_(false),
    gazebo_robot_name_("robotis")
{
  direct_sync_write_.clear();
  direct_sync_write_key_.clear();
}

void RobotisController::initializeSyncWrite()
{
  if (gazebo_mode_ == true)
    return;

  RCLCPP_INFO(this->get_logger(), "FIRST BULKREAD");
  for (auto& it : port_to_bulk_read_)
    it.second->txRxPacket();
  for(auto& it : port_to_bulk_read_)
  {
    int error_count = 0;
    int result = COMM_SUCCESS;
    do
    {
      if (++error_count > 10)
      {
        RCLCPP_ERROR(this->get_logger(), "first bulk read fail!!");
        exit(-1);
      }
      usleep(10 * 1000);
      result = it.second->txRxPacket();
    } while (result != COMM_SUCCESS);
  }
  init_pose_loaded_ = true;
  RCLCPP_INFO(this->get_logger(), "FIRST BULKREAD END");

  // clear syncwrite param setting
  for (auto& it : port_to_sync_write_position_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_position_p_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_position_i_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_position_d_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_p_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_i_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_d_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_current_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }

  // set init syncwrite param(from data of bulkread)
  for (auto& it : robot_->dxls_)
  {
    std::string joint_name = it.first;
    Dynamixel *dxl = it.second;

    for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
    {
      uint32_t  read_data = 0;
      uint8_t   sync_write_data[4];

      if (port_to_bulk_read_[dxl->port_name_]->isAvailable(dxl->id_,
                                                          dxl->bulk_read_items_[i]->address_,
                                                          dxl->bulk_read_items_[i]->data_length_) == true)
      {
        read_data = port_to_bulk_read_[dxl->port_name_]->getData(dxl->id_,
                                                                dxl->bulk_read_items_[i]->address_,
                                                                dxl->bulk_read_items_[i]->data_length_);

        sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(read_data));
        sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(read_data));
        sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(read_data));
        sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(read_data));

        if ((dxl->present_position_item_ != 0) &&
            (dxl->bulk_read_items_[i]->item_name_ == dxl->present_position_item_->item_name_))
        {
          dxl->dxl_state_->present_position_ = dxl->convertValue2Radian(read_data) - dxl->dxl_state_->position_offset_ * offset_ratio_;
          dxl->dxl_state_->goal_position_ = dxl->dxl_state_->present_position_;

          port_to_sync_write_position_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
        }
        else if ((dxl->position_p_gain_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->position_p_gain_item_->item_name_))
        {
          dxl->dxl_state_->position_p_gain_ = read_data;
        }
        else if ((dxl->position_i_gain_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->position_i_gain_item_->item_name_))
        {
          dxl->dxl_state_->position_i_gain_ = read_data;
        }
        else if ((dxl->position_d_gain_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->position_d_gain_item_->item_name_))
        {
          dxl->dxl_state_->position_d_gain_ = read_data;
        }
        else if ((dxl->present_velocity_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->present_velocity_item_->item_name_))
        {
          dxl->dxl_state_->present_velocity_ = dxl->convertValue2Velocity(read_data);
          dxl->dxl_state_->goal_velocity_ = dxl->dxl_state_->present_velocity_;
        }
        else if ((dxl->velocity_p_gain_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->velocity_p_gain_item_->item_name_))
        {
          dxl->dxl_state_->velocity_p_gain_ = read_data;
        }
        else if ((dxl->velocity_i_gain_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->velocity_i_gain_item_->item_name_))
        {
          dxl->dxl_state_->velocity_i_gain_ = read_data;
        }
        else if ((dxl->velocity_d_gain_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->velocity_d_gain_item_->item_name_))
        {
          dxl->dxl_state_->velocity_d_gain_ = read_data;
        }
        else if ((dxl->present_current_item_ != 0) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->present_current_item_->item_name_))
        {
          dxl->dxl_state_->present_torque_ = dxl->convertValue2Torque(read_data);
          dxl->dxl_state_->goal_torque_ = dxl->dxl_state_->present_torque_;
        }
      }
    }
  }
}

bool RobotisController::initialize(const std::string robot_file_path, const std::string init_file_path)
{
  std::string dev_desc_dir_path = ament_index_cpp::get_package_share_directory("robotis_device") + "/devices";

  // load robot info : port , device
  robot_ = new Robot(robot_file_path, dev_desc_dir_path);

  if (gazebo_mode_ == true)
  {
    queue_thread_ = std::thread(&RobotisController::msgQueueThread, this);
    return true;
  }

  for (auto& it : robot_->ports_)
  {
    std::string               port_name           = it.first;
    dynamixel::PortHandler   *port                = it.second;
    dynamixel::PacketHandler *default_pkt_handler = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (port->setBaudRate(port->getBaudRate()) == false)
    {
      RCLCPP_ERROR(this->get_logger(), "PORT [%s] SETUP ERROR! (baudrate: %d)", port_name.c_str(), port->getBaudRate());
      exit(-1);
    }

    // get the default device info of the port
    std::string default_device_name = robot_->port_default_device_[port_name];
    auto dxl_it = robot_->dxls_.find(default_device_name);
    auto sensor_it = robot_->sensors_.find(default_device_name);
    if (dxl_it != robot_->dxls_.end())
    {
      Dynamixel *default_device = dxl_it->second;
      default_pkt_handler = dynamixel::PacketHandler::getPacketHandler(default_device->protocol_version_);

      if (default_device->goal_position_item_ != 0)
      {
        port_to_sync_write_position_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->goal_position_item_->address_,
                                            default_device->goal_position_item_->data_length_);
      }

      if (default_device->position_p_gain_item_ != 0)
      {
        port_to_sync_write_position_p_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->position_p_gain_item_->address_,
                                            default_device->position_p_gain_item_->data_length_);
      }

      if (default_device->position_i_gain_item_ != 0)
      {
        port_to_sync_write_position_i_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->position_i_gain_item_->address_,
                                            default_device->position_i_gain_item_->data_length_);
      }

      if (default_device->position_d_gain_item_ != 0)
      {
        port_to_sync_write_position_d_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->position_d_gain_item_->address_,
                                            default_device->position_d_gain_item_->data_length_);
      }

      if (default_device->goal_velocity_item_ != 0)
      {
        port_to_sync_write_velocity_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->goal_velocity_item_->address_,
                                            default_device->goal_velocity_item_->data_length_);
      }

      if (default_device->velocity_p_gain_item_ != 0)
      {
        port_to_sync_write_velocity_p_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->velocity_p_gain_item_->address_,
                                            default_device->velocity_p_gain_item_->data_length_);
      }

      if (default_device->velocity_i_gain_item_ != 0)
      {
        port_to_sync_write_velocity_i_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->velocity_i_gain_item_->address_,
                                            default_device->velocity_i_gain_item_->data_length_);
      }

      if (default_device->velocity_d_gain_item_ != 0)
      {
        port_to_sync_write_velocity_d_gain_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->velocity_d_gain_item_->address_,
                                            default_device->velocity_d_gain_item_->data_length_);
      }

      if (default_device->goal_current_item_ != 0)
      {
        port_to_sync_write_current_[port_name]
            = new dynamixel::GroupSyncWrite(port,
                                            default_pkt_handler,
                                            default_device->goal_current_item_->address_,
                                            default_device->goal_current_item_->data_length_);
      }
    }
    else if (sensor_it != robot_->sensors_.end())
    {
      Sensor *_default_device = sensor_it->second;
      default_pkt_handler = dynamixel::PacketHandler::getPacketHandler(_default_device->protocol_version_);
    }

    port_to_bulk_read_[port_name] = new dynamixel::GroupBulkRead(port, default_pkt_handler);
  }

  // (for loop) check all dxls are connected.
  for (auto& it : robot_->dxls_)
  {
    std::string joint_name  = it.first;
    Dynamixel  *dxl         = it.second;

    if (ping(joint_name) != 0)
    {
      usleep(10 * 1000);
      if (ping(joint_name) != 0)
        RCLCPP_ERROR(this->get_logger(), "JOINT[%s] does NOT respond!!", joint_name.c_str());
    }
  }

  initializeDevice(init_file_path);

  queue_thread_ = std::thread(&RobotisController::msgQueueThread, this);
  return true;
}

void RobotisController::initializeDevice(const std::string init_file_path)
{
  // device initialize
  if (DEBUG_PRINT)
    RCLCPP_WARN(this->get_logger(), "INIT FILE LOAD");

  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(init_file_path.c_str());

    for (YAML::const_iterator it_doc = doc.begin(); it_doc != doc.end(); it_doc++)
    {
      std::string joint_name = it_doc->first.as<std::string>();

      YAML::Node joint_node = doc[joint_name];
      if (joint_node.size() == 0)
        continue;

      Dynamixel *dxl = NULL;
      auto dxl_it = robot_->dxls_.find(joint_name);
      if (dxl_it != robot_->dxls_.end())
        dxl = dxl_it->second;

      if (dxl == NULL)
      {
        RCLCPP_WARN(this->get_logger(), "Joint [%s] was not found.", joint_name.c_str());
        continue;
      }
      if (DEBUG_PRINT)
        RCLCPP_INFO(this->get_logger(), "JOINT_NAME: %s", joint_name.c_str());

      uint8_t torque_enabled = 0;
      read1Byte(joint_name, dxl->torque_enable_item_->address_, &torque_enabled);

      for (YAML::const_iterator it_joint = joint_node.begin(); it_joint != joint_node.end(); it_joint++)
      {
        std::string item_name = it_joint->first.as<std::string>();

        if (DEBUG_PRINT)
          RCLCPP_INFO(this->get_logger(), "  ITEM_NAME: %s", item_name.c_str());

        uint32_t value = it_joint->second.as<uint32_t>();

        ControlTableItem *item = dxl->ctrl_table_[item_name];
        if (item == NULL)
        {
          RCLCPP_WARN(this->get_logger(), "Control Item [%s] was not found.", item_name.c_str());
          continue;
        }

        if (item->memory_type_ == EEPROM)
        {
          uint8_t   data8 = 0;
          uint16_t  data16 = 0;
          uint32_t  data32 = 0;

          switch (item->data_length_)
          {
          case 1:
            read1Byte(joint_name, item->address_, &data8);
            if (data8 == value)
              continue;
            break;
          case 2:
            read2Byte(joint_name, item->address_, &data16);
            if (data16 == value)
              continue;
            break;
          case 4:
            read4Byte(joint_name, item->address_, &data32);
            if (data32 == value)
              continue;
            break;
          default:
            break;
          }

          if (torque_enabled == 1)
          {
              RCLCPP_ERROR(this->get_logger(), "################\nThe initial value of the EEPROM area has been changed. \nTurn off Torque Enable and try again.");
              exit(-1);
          }
        }

        switch (item->data_length_)
        {
        case 1:
          write1Byte(joint_name, item->address_, (uint8_t) value);
          break;
        case 2:
          write2Byte(joint_name, item->address_, (uint16_t) value);
          break;
        case 4:
          write4Byte(joint_name, item->address_, value);
          break;
        default:
          break;
        }

        if (item->memory_type_ == EEPROM)
        {
          // Write to EEPROM -> delay is required (max delay: 55 msec per byte)
          usleep(item->data_length_ * 55 * 1000);
        }
      }
    }
  } catch (const std::exception& e)
  {
    RCLCPP_INFO(this->get_logger(), "Dynamixel Init file not found.");
  }

  // [ BulkRead ] StartAddress : Present Position , Length : 10 ( Position/Velocity/Current )
  for (auto& it : robot_->ports_)
  {
    if (port_to_bulk_read_[it.first] != 0)
      port_to_bulk_read_[it.first]->clearParam();
  }
  for (auto& it : robot_->dxls_)
  {
    std::string joint_name  = it.first;
    Dynamixel  *dxl         = it.second;

    if (dxl == NULL)
      continue;

    int bulkread_start_addr = 0;
    int bulkread_data_length = 0;

    uint8_t torque_enabled = 0;
    read1Byte(joint_name, dxl->torque_enable_item_->address_, &torque_enabled);

    // calculate bulk read start address & data length
    auto indirect_addr_it = dxl->ctrl_table_.find(INDIRECT_ADDRESS_1);
    if (indirect_addr_it != dxl->ctrl_table_.end()) // INDIRECT_ADDRESS_1 exist
    {
      if (dxl->bulk_read_items_.size() != 0)
      {
        bulkread_start_addr = dxl->bulk_read_items_[0]->address_;

        // set indirect address
        int indirect_addr = indirect_addr_it->second->address_;

        // Each mapped byte needs one 2-byte indirect entry. Reading them back
        // one-by-one (read2Byte per byte) was the dominant boot cost: ~15 ms per
        // round-trip x ~200 entries on this OpenCR USB bus. Instead read the
        // whole indirect block in ONE transaction, compare in memory, and write
        // only the (normally zero) entries that differ. Behavior is identical.
        int total_bytes = 0;
        for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
          total_bytes += dxl->bulk_read_items_[i]->data_length_;
        bulkread_data_length = total_bytes;

        uint8_t indirect_buf[128] = {0, };
        bool have_block = false;
        if (total_bytes * 2 <= (int) sizeof(indirect_buf))
        {
          dynamixel::PacketHandler *pkt  = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
          dynamixel::PortHandler   *port = robot_->ports_[dxl->port_name_];
          if (pkt->readTxRx(port, dxl->id_, indirect_addr,
                            (uint16_t)(total_bytes * 2), indirect_buf) == COMM_SUCCESS)
            have_block = true;
        }

        int entry = 0;  // running index of 2-byte indirect entries
        for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
        {
          int addr_leng = dxl->bulk_read_items_[i]->data_length_;
          for (int l = 0; l < addr_leng; l++)
          {
            uint16_t expected = dxl->ctrl_table_[dxl->bulk_read_items_[i]->item_name_]->address_ + l;
            // If the block read failed, force a rewrite (0xFFFF never matches).
            uint16_t current  = have_block
              ? (uint16_t)(indirect_buf[entry * 2] | (indirect_buf[entry * 2 + 1] << 8))
              : (uint16_t) 0xFFFF;
            if (current != expected)
            {
              if (torque_enabled == 1)
              {
                RCLCPP_ERROR(this->get_logger(), "################\nThe indirect address of the EEPROM area has been changed. \nTurn off Torque Enable and try again.");
                exit(-1);
              }
              write2Byte(joint_name, indirect_addr + entry * 2, expected);
            }
            entry++;
          }
        }
      }
    }
    else    // INDIRECT_ADDRESS_1 NOT exist
    {
      if (dxl->bulk_read_items_.size() != 0)
      {
        bulkread_start_addr = dxl->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        ControlTableItem *last_item = dxl->bulk_read_items_[0];

        for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
        {
          int addr = dxl->bulk_read_items_[i]->address_;
          if (addr < bulkread_start_addr)
            bulkread_start_addr = addr;
          else if (last_item->address_ < addr)
            last_item = dxl->bulk_read_items_[i];
        }

        bulkread_data_length = last_item->address_ - bulkread_start_addr + last_item->data_length_;
      }
    }

    if (bulkread_start_addr != 0)
      port_to_bulk_read_[dxl->port_name_]->addParam(dxl->id_, bulkread_start_addr, bulkread_data_length);

    // Torque ON
    if (writeCtrlItem(joint_name, dxl->torque_enable_item_->item_name_, 1) != COMM_SUCCESS)
      writeCtrlItem(joint_name, dxl->torque_enable_item_->item_name_, 1);
  }

  for (auto& it : robot_->sensors_)
  {
    std::string sensor_name = it.first;
    Sensor     *sensor      = it.second;

    if (sensor == NULL)
      continue;

    int bulkread_start_addr = 0;
    int bulkread_data_length = 0;

    // calculate bulk read start address & data length
    auto indirect_addr_it = sensor->ctrl_table_.find(INDIRECT_ADDRESS_1);
    if (indirect_addr_it != sensor->ctrl_table_.end()) // INDIRECT_ADDRESS_1 exist
    {
      if (sensor->bulk_read_items_.size() != 0)
      {
        uint16_t  data16 = 0;

        bulkread_start_addr = sensor->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        // set indirect address
        int indirect_addr = indirect_addr_it->second->address_;
        for (int i = 0; i < sensor->bulk_read_items_.size(); i++)
        {
          int addr_leng = sensor->bulk_read_items_[i]->data_length_;

          bulkread_data_length += addr_leng;
          for (int l = 0; l < addr_leng; l++)
          {
            read2Byte(sensor_name, indirect_addr, &data16);
            if (data16 != sensor->ctrl_table_[sensor->bulk_read_items_[i]->item_name_]->address_ + l)
            {
              write2Byte(sensor_name,
                         indirect_addr,
                         sensor->ctrl_table_[sensor->bulk_read_items_[i]->item_name_]->address_ + l);
            }
            indirect_addr += 2;
          }
        }
      }
    }
    else    // INDIRECT_ADDRESS_1 NOT exist
    {
      if (sensor->bulk_read_items_.size() != 0)
      {
        bulkread_start_addr = sensor->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        ControlTableItem *last_item = sensor->bulk_read_items_[0];

        for (int i = 0; i < sensor->bulk_read_items_.size(); i++)
        {
          int addr = sensor->bulk_read_items_[i]->address_;
          if (addr < bulkread_start_addr)
            bulkread_start_addr = addr;
          else if (last_item->address_ < addr)
            last_item = sensor->bulk_read_items_[i];
        }

        bulkread_data_length = last_item->address_ - bulkread_start_addr + last_item->data_length_;
      }
    }

    if (bulkread_start_addr != 0)
      port_to_bulk_read_[sensor->port_name_]->addParam(sensor->id_, bulkread_start_addr, bulkread_data_length);
  }
}

void RobotisController::gazeboTimerThread()
{
  rclcpp::Rate gazebo_rate(1000 / robot_->getControlCycle());

  while (!stop_timer_)
  {
    if (init_pose_loaded_ == true)
      process();
    gazebo_rate.sleep();
  }
}

void RobotisController::msgQueueThread()
{
  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(this->get_node_base_interface());

  /* subscriber */
  auto write_control_table_sub = this->create_subscription<robotis_controller_msgs::msg::WriteControlTable>(
      "/robotis/write_control_table", 5, std::bind(&RobotisController::writeControlTableCallback, this, std::placeholders::_1));
  auto sync_write_item_sub = this->create_subscription<robotis_controller_msgs::msg::SyncWriteItem>(
      "/robotis/sync_write_item", 10, std::bind(&RobotisController::syncWriteItemCallback, this, std::placeholders::_1));
  auto joint_ctrl_modules_sub = this->create_subscription<robotis_controller_msgs::msg::JointCtrlModule>(
      "/robotis/set_joint_ctrl_modules", 10, std::bind(&RobotisController::setJointCtrlModuleCallback, this, std::placeholders::_1));
  auto enable_ctrl_module_sub = this->create_subscription<std_msgs::msg::String>(
      "/robotis/enable_ctrl_module", 10, std::bind(&RobotisController::setCtrlModuleCallback, this, std::placeholders::_1));
  auto control_mode_sub = this->create_subscription<std_msgs::msg::String>(
      "/robotis/set_control_mode", 10, std::bind(&RobotisController::setControllerModeCallback, this, std::placeholders::_1));
  auto joint_states_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "/robotis/set_joint_states", 10, std::bind(&RobotisController::setJointStatesCallback, this, std::placeholders::_1));
  auto enable_offset_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/robotis/enable_offset", 10, std::bind(&RobotisController::enableOffsetCallback, this, std::placeholders::_1));

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gazebo_joint_states_sub;
  if (gazebo_mode_ == true)
  {
    gazebo_joint_states_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "/" + gazebo_robot_name_ + "/joint_states", 10, std::bind(&RobotisController::gazeboJointStatesCallback, this, std::placeholders::_1));
  }

  /* publisher */
  goal_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/robotis/goal_joint_states", 10);
  present_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/robotis/present_joint_states", 10);
  current_module_pub_ = this->create_publisher<robotis_controller_msgs::msg::JointCtrlModule>("/robotis/present_joint_ctrl_modules", 10);

  if (gazebo_mode_ == true)
  {
    for (auto& it : robot_->dxls_)
    {
      gazebo_joint_position_pub_[it.first] = this->create_publisher<std_msgs::msg::Float64>(
          "/" + gazebo_robot_name_ + "/" + it.first + "_position/command", 1);
      gazebo_joint_velocity_pub_[it.first] = this->create_publisher<std_msgs::msg::Float64>(
          "/" + gazebo_robot_name_ + "/" + it.first + "_velocity/command", 1);
      gazebo_joint_effort_pub_[it.first] = this->create_publisher<std_msgs::msg::Float64>(
          "/" + gazebo_robot_name_ + "/" + it.first + "_effort/command", 1);
    }
  }

  /* service */
  auto get_joint_module_server = this->create_service<robotis_controller_msgs::srv::GetJointModule>(
      "/robotis/get_present_joint_ctrl_modules", std::bind(&RobotisController::getJointCtrlModuleService, this, std::placeholders::_1, std::placeholders::_2));
  auto set_joint_module_server = this->create_service<robotis_controller_msgs::srv::SetJointModule>(
      "/robotis/set_present_joint_ctrl_modules", std::bind(&RobotisController::setJointCtrlModuleService, this, std::placeholders::_1, std::placeholders::_2));
  auto set_module_server = this->create_service<robotis_controller_msgs::srv::SetModule>(
      "/robotis/set_present_ctrl_modules", std::bind(&RobotisController::setCtrlModuleService, this, std::placeholders::_1, std::placeholders::_2));
  auto load_offset_server = this->create_service<robotis_controller_msgs::srv::LoadOffset>(
      "/robotis/load_offset", std::bind(&RobotisController::loadOffsetService, this, std::placeholders::_1, std::placeholders::_2));

  executor.spin();
}

void *RobotisController::timerThread(void *param)
{
  RobotisController *controller = (RobotisController *) param;
  static struct timespec next_time;
  static struct timespec curr_time;

  RCLCPP_DEBUG(controller->get_logger(), "controller::thread_proc started");

  clock_gettime(CLOCK_MONOTONIC, &next_time);

  while (!controller->stop_timer_)
  {
    next_time.tv_sec += (next_time.tv_nsec + controller->robot_->getControlCycle() * 1000000) / 1000000000;
    next_time.tv_nsec = (next_time.tv_nsec + controller->robot_->getControlCycle() * 1000000) % 1000000000;

    controller->process();

    clock_gettime(CLOCK_MONOTONIC, &curr_time);
    long delta_nsec = (next_time.tv_sec - curr_time.tv_sec) * 1000000000 + (next_time.tv_nsec - curr_time.tv_nsec);
    if (delta_nsec < -100000)
    {
      if (controller->DEBUG_PRINT == true)
      {
        fprintf(stderr, "[RobotisController::ThreadProc] NEXT TIME < CURR TIME.. (%f)[%ld.%09ld / %ld.%09ld]",
                         delta_nsec / 1000000.0, (long )next_time.tv_sec, (long )next_time.tv_nsec,
                         (long )curr_time.tv_sec, (long )curr_time.tv_nsec);
      }

      // next_time = curr_time + 3 msec
      next_time.tv_sec = curr_time.tv_sec + (curr_time.tv_nsec + 3000000) / 1000000000;
      next_time.tv_nsec = (curr_time.tv_nsec + 3000000) % 1000000000;
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
  return 0;
}

void RobotisController::startTimer()
{
  if (this->is_timer_running_ == true)
    return;

  if (this->gazebo_mode_ == true)
  {
    // create and start the thread
    gazebo_thread_ = std::thread(&RobotisController::gazeboTimerThread, this);
  }
  else
  {
    initializeSyncWrite();

    for (auto& it : port_to_bulk_read_)
    {
      it.second->txPacket();
    }

    usleep(8 * 1000);

    int error;
    struct sched_param param;
    pthread_attr_t attr;

    pthread_attr_init(&attr);

    error = pthread_attr_setschedpolicy(&attr, SCHED_RR);
    if (error != 0)
      RCLCPP_ERROR(this->get_logger(), "pthread_attr_setschedpolicy error = %d\n", error);
    error = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (error != 0)
      RCLCPP_ERROR(this->get_logger(), "pthread_attr_setinheritsched error = %d\n", error);

    memset(&param, 0, sizeof(param));
    param.sched_priority = 31;    // RT
    error = pthread_attr_setschedparam(&attr, &param);
    if (error != 0)
      RCLCPP_ERROR(this->get_logger(), "pthread_attr_setschedparam error = %d\n", error);

    // create and start the thread
    if ((error = pthread_create(&this->timer_thread_, &attr, this->timerThread, this)) != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Creating timer thread failed!!");
      exit(-1);
    }
  }

  this->is_timer_running_ = true;
}

void RobotisController::stopTimer()
{
  int error = 0;

  // set the flag to stop the thread
  if (this->is_timer_running_)
  {
    this->stop_timer_ = true;

    if (this->gazebo_mode_ == false)
    {
      // wait until the thread is stopped.
      if ((error = pthread_join(this->timer_thread_, NULL)) != 0)
        exit(-1);

      for (auto& it : port_to_bulk_read_)
      {
        if (it.second != NULL)
          it.second->rxPacket();
      }

      for (auto& it : port_to_sync_write_position_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_position_p_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_position_i_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_position_d_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_velocity_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_velocity_p_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_velocity_i_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_velocity_d_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_current_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
    }
    else
    {
      // wait until the thread is stopped.
      gazebo_thread_.join();
    }

    this->stop_timer_ = false;
    this->is_timer_running_ = false;
  }
}

bool RobotisController::isTimerRunning()
{
  return this->is_timer_running_;
}

void RobotisController::loadOffset(const std::string path)
{
  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    RCLCPP_WARN(this->get_logger(), "Fail to load offset yaml.");
    return;
  }

  YAML::Node offset_node = doc["offset"];
  if (offset_node.size() == 0)
    return;

  RCLCPP_INFO(this->get_logger(), "Load offsets...");
  for (YAML::const_iterator it = offset_node.begin(); it != offset_node.end(); it++)
  {
    std::string joint_name = it->first.as<std::string>();
    double offset = it->second.as<double>();

    auto dxl_it = robot_->dxls_.find(joint_name);
    if (dxl_it != robot_->dxls_.end())
      dxl_it->second->dxl_state_->position_offset_ = offset;
  }
}

// ---------------------------------------------------------------------------
// Profil loop kontrol. Mati total kecuali env OP3_PROFILE=1 diset, jadi aman
// ditinggal di kode. Dipakai melacak gerakan tersendat: yang mau dijawab adalah
// "bulk read gagal karena datanya belum sampai, atau karena waktunya habis di
// tempat lain?".
// ---------------------------------------------------------------------------
static inline double op3_prof_now_ms()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000.0 + ts.tv_nsec * 1e-6;
}

struct Op3ProfPort
{
  long ok = 0, fail = 0;
  double rx_sum = 0.0, rx_max = 0.0;
  long avail_sum = 0, avail_max = 0, avail_zero = 0;   // byte menganggur saat Rx mulai
  long tx_ok = 0, tx_busy = 0, tx_other = 0;           // hasil BulkRead Tx
  int  last_rx_result = 0;
  long avail_presw_sum = 0;                            // byte sesaat sebelum sync write
  long sw_ok = 0, sw_busy = 0, sw_other = 0;           // hasil sync write posisi
};

void RobotisController::process()
{
  // avoid duplicated function call
  static bool is_process_running = false;
  if (is_process_running == true)
    return;
  is_process_running = true;

  // ROS_INFO("Controller::Process()");
  // offset ratio
  if(is_offset_enabled_)
  {
    if(offset_ratio_ < 1.0)
      offset_ratio_ += 0.01;
    else
      offset_ratio_ = 1.0;    
  }
  else
  {
    if(offset_ratio_ > 0.0)
      offset_ratio_ -= 0.01;
    else
      offset_ratio_ = 0.0;    
  }
  

  rclcpp::Time start_time;

  if (DEBUG_PRINT)
    start_time = rclcpp::Clock().now();

  // ---------------------------------------------------------------------------
  // Port yang isinya HANYA sensor (di ALPHONSE: OpenCR lewat USB CDC, karena
  // transceiver TTL board-nya mati). Dipisahkan supaya transaksinya bisa
  // dituntaskan di AWAL siklus -- lihat alasan panjangnya di bawah.
  // Pada robot yang OpenCR-nya ikut bus servo (satu port, susunan OP3 normal)
  // himpunan ini KOSONG dan tidak ada perilaku yang berubah.
  // ---------------------------------------------------------------------------
  // Berapa siklus sekali port sensor terpisah dibaca. 4 = 31 Hz pada
  // control_cycle 8 ms; cukup untuk IMU (deteksi jatuh) dan tombol, yang tidak
  // butuh 125 Hz. Menurunkan laju ini mengurangi rebutan slot USB dengan U2D2.
  // Terukur, sambil kepala menyapu (rx_ok bulk read servo / laju perubahan
  // head_pan di /robotis/present_joint_states):
  //   div 1 ... 34-92 %,  88 Hz
  //   div 4 ... 77-97 %, 104 Hz   <-- dipakai
  //   div 8 ... 58-98 %,  96 Hz
  // Bisa ditimpa saat jalan lewat OP3_SENSOR_DIV.
  static const int sensor_div = (getenv("OP3_SENSOR_DIV") != NULL)
                                ? std::max(1, atoi(getenv("OP3_SENSOR_DIV"))) : 4;
  static long sensor_tick = 0;
  const bool do_sensor = (sensor_tick++ % sensor_div) == 0;

  static std::map<std::string, Op3ProfPort> prof_port_sensor;
  static std::set<std::string> sensor_only_ports;
  static bool sensor_only_scanned = false;
  if (sensor_only_scanned == false)
  {
    sensor_only_scanned = true;
    for (auto& it : port_to_bulk_read_)
    {
      bool has_dxl = false;
      for (auto& d : robot_->dxls_)
      {
        if (d.second != NULL && d.second->port_name_ == it.first)
        {
          has_dxl = true;
          break;
        }
      }
      if (has_dxl == false)
      {
        sensor_only_ports.insert(it.first);
        RCLCPP_INFO(this->get_logger(),
                    "Port sensor terpisah: %s -- bulk read-nya dijalankan di awal siklus",
                    it.first.c_str());
      }
    }
  }

  sensor_msgs::msg::JointState goal_state;
  sensor_msgs::msg::JointState present_state;

  present_state.header.stamp = rclcpp::Clock().now();
  goal_state.header.stamp = present_state.header.stamp;

  if (controller_mode_ == MotionModuleMode)
  {
    if (gazebo_mode_ == false)
    {
      // -----------------------------------------------------------------------
      // Transaksi port sensor terpisah dituntaskan di SINI, di awal siklus,
      // Rx dan Tx sekaligus -- bukan ikut antre bersama port servo.
      //
      // Alasannya bukan soal ROS, tapi soal USB. Di robot ini OpenCR adalah
      // perangkat full-speed (12 Mbit) yang menempel di hub 480 Mbit yang SAMA
      // dengan U2D2. Perangkat 12 Mbit di balik hub high-speed memaksa host
      // memakai split transaction, dan slot itu diambil dari jatah microframe
      // yang juga dipakai U2D2. Kalau transaksi OpenCR jatuh bersamaan dengan
      // saat 420 byte balasan 20 servo sedang mengalir masuk, sebagian byte itu
      // HILANG -- bukan telat, hilang.
      //
      // Terukur (loop 125 Hz identik di luar ROS, agar bebas dari dugaan soal
      // ROS/CPU):
      //   port OpenCR dibuka tapi tak disentuh ....... 100 %  (420/420 byte)
      //   disentuh tiap siklus, Tx di akhir .......... 3 %    (~300/420 byte)
      //   disentuh tiap 2 siklus ..................... 54 %
      //   disentuh tiap 4 siklus ..................... 78 %
      //   disentuh tiap 8 siklus ..................... 90 %
      //   disentuh tiap siklus, transaksi di AWAL .... 100 %  <-- yang dipakai
      //
      // Pola "tiap N siklus" membuktikan sebabnya: kegagalan jatuh persis pada
      // siklus yang menyentuh OpenCR. Memindahkan transaksinya ke awal siklus
      // menaruh lalu lintas USB-nya di jendela mati -- saat itu balasan servo
      // siklus sebelumnya sudah lama lengkap, dan permintaan berikutnya belum
      // dikirim -- sehingga jendela balasan servo bersih sepenuhnya.
      //
      // Kalau ini dikembalikan ke satu loop bersama port servo, gerakan kepala
      // akan patah-patah lagi.
      // -----------------------------------------------------------------------
      for (auto& it : port_to_bulk_read_)
      {
        if (sensor_only_ports.count(it.first) == 0 || do_sensor == false)
          continue;
        robot_->ports_[it.first]->setPacketTimeout(1.0);
        if (getenv("OP3_PROFILE") != NULL)
        {
          Op3ProfPort &pp = prof_port_sensor[it.first];
          const int av = robot_->ports_[it.first]->getBytesAvailable();
          pp.avail_sum += av;
          if (av > pp.avail_max) pp.avail_max = av;
          if (av == 0) pp.avail_zero++;
          if (it.second->rxPacket() == COMM_SUCCESS) pp.ok++; else pp.fail++;
        }
        else
          it.second->rxPacket();
        it.second->txPacket();
      }

      static const bool prof_on = (getenv("OP3_PROFILE") != NULL);
      static std::map<std::string, Op3ProfPort> prof_port;
      static double prof_prev_tx_ms = 0.0;
      static double prof_report_ms  = 0.0;
      static long   prof_cycles     = 0;
      static double prof_gap_sum = 0.0, prof_gap_min = 1e9;
      static double prof_proc_sum = 0.0, prof_proc_max = 0.0;
      static double prof_mod_sum = 0.0, prof_mod_max = 0.0;
      static double prof_sw_sum = 0.0, prof_sw_max = 0.0;
      const double prof_t_begin = prof_on ? op3_prof_now_ms() : 0.0;
      if (prof_on && prof_prev_tx_ms > 0.0)
      {
        const double gap = prof_t_begin - prof_prev_tx_ms;
        prof_gap_sum += gap;
        if (gap < prof_gap_min) prof_gap_min = gap;
      }

      // BulkRead Rx
      for (auto& it : port_to_bulk_read_)
      {
        if (sensor_only_ports.count(it.first) != 0)
          continue;   // sudah diurus di awal siklus

        // Anggaran tunggu Rx. Nilainya KECIL dengan sengaja -- ini bukan sekadar
        // "berapa lama boleh menunggu", tapi penentu apakah loop ini stabil.
        //
        // Bulk read di sini dipipa: permintaan dikirim di AKHIR process(),
        // balasannya dibaca di AWAL process() berikutnya. Jadi waktu yang
        // dipunyai servo untuk menjawab = control_cycle - lama process(). Kalau
        // Rx gagal lalu menunggu lama, process() jadi panjang, jeda untuk
        // balasan BERIKUTNYA menyusut, dan kegagalan berikutnya jadi lebih
        // mungkin. Umpan balik positif: sekali tersandung, loop terkunci gagal.
        //
        // Terukur di robot ini (2026-08-25, OP3_PROFILE=1): dengan timeout 3 ms
        // manager mulai sehat (rx_ok 20%) lalu RUNTUH ke 0% dalam ~8 detik dan
        // tidak pernah pulih. Sebabnya persis di atas: 3 ms tunggu + 0,15 ms
        // hitung = jeda tinggal 4,87 ms, sedangkan balasan 20 servo baru
        // lengkap ~4,5 ms setelah write() (3,88 ms di kabel + antrean tulis
        // sync-write yang mendahuluinya). Sisa 0,4 ms -- terlalu tipis.
        //
        // Dengan 1 ms: jeda 6,85 ms lawan kebutuhan 4,5 ms. Kegagalan sesekali
        // tetap termaafkan, tapi tidak bisa lagi memakan jeda sampai terkunci.
        //
        // Kalau ini dinaikkan lagi, gerakan kepala akan patah-patah lagi.
        // Bukti tandingan yang penting: loop 125 Hz yang sama persis di luar ROS
        // (scripts/op3_bus_loop.py) berhasil 100% selama 20 detik -- bus, servo,
        // U2D2, dan latency_timer=1 semuanya SEHAT. Yang salah cuma anggaran
        // waktunya.
        robot_->ports_[it.first]->setPacketTimeout(1.0);
        const int prof_avail = prof_on ? robot_->ports_[it.first]->getBytesAvailable() : 0;
        const double prof_t_rx = prof_on ? op3_prof_now_ms() : 0.0;
        int result = it.second->rxPacket();
        if (prof_on)
        {
          Op3ProfPort &pp = prof_port[it.first];
          pp.avail_sum += prof_avail;
          if (prof_avail > pp.avail_max) pp.avail_max = prof_avail;
          if (prof_avail == 0) pp.avail_zero++;
          pp.last_rx_result = result;
          const double d = op3_prof_now_ms() - prof_t_rx;
          pp.rx_sum += d;
          if (d > pp.rx_max) pp.rx_max = d;
          if (result == COMM_SUCCESS) pp.ok++; else pp.fail++;
        }
        if (DEBUG_PRINT && result != COMM_SUCCESS)
          RCLCPP_ERROR(this->get_logger(), "Bulk Read Fail : %s", it.first.c_str());
      }

      // -> save to robot->dxls_[]->dxl_state_
      if (robot_->dxls_.size() > 0)
      {
        for (auto& dxl_it : robot_->dxls_)
        {
          Dynamixel  *dxl         = dxl_it.second;
          std::string port_name   = dxl_it.second->port_name_;
          std::string joint_name  = dxl_it.first;

          if (dxl->bulk_read_items_.size() > 0)
          {
            bool      updated = false;
            uint32_t  data    = 0;
            for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
            {
              ControlTableItem *item = dxl->bulk_read_items_[i];
              if (port_to_bulk_read_[port_name]->isAvailable(dxl->id_, item->address_, item->data_length_) == true)
              {
                updated = true;
                data = port_to_bulk_read_[port_name]->getData(dxl->id_, item->address_, item->data_length_);

                // change dxl_state
                if (dxl->present_position_item_ != 0 && item->item_name_ == dxl->present_position_item_->item_name_)
                {
                  dxl->dxl_state_->present_position_ = dxl->convertValue2Radian(data) - dxl->dxl_state_->position_offset_ * offset_ratio_;
                }
                else if (dxl->present_velocity_item_ != 0 && item->item_name_ == dxl->present_velocity_item_->item_name_)
                  dxl->dxl_state_->present_velocity_ = dxl->convertValue2Velocity(data);
                else if (dxl->present_current_item_ != 0 && item->item_name_ == dxl->present_current_item_->item_name_)
                  dxl->dxl_state_->present_torque_ = dxl->convertValue2Torque(data);
                else if (dxl->goal_position_item_ != 0 && item->item_name_ == dxl->goal_position_item_->item_name_)
                {
                  dxl->dxl_state_->goal_position_ = dxl->convertValue2Radian(data) - dxl->dxl_state_->position_offset_ * offset_ratio_;
                }
                else if (dxl->goal_velocity_item_ != 0 && item->item_name_ == dxl->goal_velocity_item_->item_name_)
                  dxl->dxl_state_->goal_velocity_ = dxl->convertValue2Velocity(data);
                else if (dxl->goal_current_item_ != 0 && item->item_name_ == dxl->goal_current_item_->item_name_)
                  dxl->dxl_state_->goal_torque_ = dxl->convertValue2Torque(data);

                dxl->dxl_state_->bulk_read_table_[item->item_name_] = data;
              }
            }

            // -> update time stamp to Robot->dxls[]->dynamixel_state.update_time_stamp
            if (updated == true)
              dxl->dxl_state_->update_time_stamp_ = TimeStamp(present_state.header.stamp.sec, present_state.header.stamp.nanosec);
          }
        }
      }

      // -> save to robot->sensors_[]->sensor_state_
      if (robot_->sensors_.size() > 0)
      {
        for (auto& sensor_it : robot_->sensors_)
        {
          Sensor     *sensor      = sensor_it.second;
          std::string port_name   = sensor_it.second->port_name_;
          std::string sensor_name = sensor_it.first;

          if (sensor->bulk_read_items_.size() > 0)
          {
            bool      updated = false;
            uint32_t  data    = 0;
            for (int i = 0; i < sensor->bulk_read_items_.size(); i++)
            {
              ControlTableItem *item = sensor->bulk_read_items_[i];
              if (port_to_bulk_read_[port_name]->isAvailable(sensor->id_, item->address_, item->data_length_) == true)
              {
                updated = true;
                data = port_to_bulk_read_[port_name]->getData(sensor->id_, item->address_, item->data_length_);

                // change sensor_state
                sensor->sensor_state_->bulk_read_table_[item->item_name_] = data;
              }
            }

            // -> update time stamp to Robot->dxls[]->dynamixel_state.update_time_stamp
            if (updated == true)
              sensor->sensor_state_->update_time_stamp_ = TimeStamp(present_state.header.stamp.sec, present_state.header.stamp.nanosec);
          }
        }
      }

      if (DEBUG_PRINT)
      {
        rclcpp::Duration time_duration = rclcpp::Clock().now() - start_time;
        fprintf(stderr, "(%2.6f) BulkRead Rx & update state \n", time_duration.nanoseconds() * 0.000001);
      }

      const double prof_t_mod_end = prof_on ? op3_prof_now_ms() : 0.0;
      if (prof_on)
      {
        for (auto& it : port_to_bulk_read_)
        {
          if (sensor_only_ports.count(it.first) != 0)
            continue;   // port sensor punya baris laporannya sendiri
          prof_port[it.first].avail_presw_sum += robot_->ports_[it.first]->getBytesAvailable();
        }
      }

      // SyncWrite
      queue_mutex_.lock();

      if (direct_sync_write_.size() > 0)
      {
        for (int i = 0; i < direct_sync_write_.size(); i++)
        {
          direct_sync_write_[i]->txPacket();
          direct_sync_write_[i]->clearParam();
        }
        direct_sync_write_.clear();
        direct_sync_write_key_.clear();
      }

      if (port_to_sync_write_position_p_gain_.size() > 0)
      {
        for (auto& it : port_to_sync_write_position_p_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }
      if (port_to_sync_write_position_i_gain_.size() > 0)
      {
        for (auto& it : port_to_sync_write_position_i_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }
      if (port_to_sync_write_position_d_gain_.size() > 0)
      {
        for (auto& it : port_to_sync_write_position_d_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }
      if (port_to_sync_write_velocity_p_gain_.size() > 0)
      {
        for (auto& it : port_to_sync_write_velocity_p_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }
      if (port_to_sync_write_velocity_i_gain_.size() > 0)
      {
        for (auto& it : port_to_sync_write_velocity_i_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }
      if (port_to_sync_write_velocity_d_gain_.size() > 0)
      {
        for (auto& it : port_to_sync_write_velocity_d_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }
      for (auto& it : port_to_sync_write_position_)
      {
        if (it.second != NULL)
        {
          int r = it.second->txPacket();
          if (prof_on)
          {
            Op3ProfPort &pp = prof_port[it.first];
            if (r == COMM_SUCCESS)        pp.sw_ok++;
            else if (r == COMM_PORT_BUSY) pp.sw_busy++;
            else                          pp.sw_other++;
          }
        }
      }
      for (auto& it : port_to_sync_write_velocity_)
      {
        if (it.second != NULL)
          it.second->txPacket();
      }
      for (auto& it : port_to_sync_write_current_)
      {
        if (it.second != NULL)
          it.second->txPacket();
      }

      queue_mutex_.unlock();

      const double prof_t_sw_end = prof_on ? op3_prof_now_ms() : 0.0;

      // BulkRead Tx
      for (auto& it : port_to_bulk_read_)
      {
        if (sensor_only_ports.count(it.first) != 0)
          continue;   // sudah dikirim di awal siklus

        int tx_result = it.second->txPacket();
        if (prof_on)
        {
          Op3ProfPort &pp = prof_port[it.first];
          if (tx_result == COMM_SUCCESS)         pp.tx_ok++;
          else if (tx_result == COMM_PORT_BUSY)  pp.tx_busy++;
          else                                   pp.tx_other++;
        }
      }

      if (prof_on)
      {
        const double now = op3_prof_now_ms();
        prof_prev_tx_ms = now;
        prof_cycles++;

        const double mod = prof_t_mod_end - prof_t_begin;   // Rx + hitung modul
        const double sw  = prof_t_sw_end  - prof_t_mod_end; // seluruh sync write
        const double pr  = now - prof_t_begin;              // satu process()
        prof_mod_sum += mod; if (mod > prof_mod_max) prof_mod_max = mod;
        prof_sw_sum  += sw;  if (sw  > prof_sw_max)  prof_sw_max  = sw;
        prof_proc_sum += pr; if (pr > prof_proc_max) prof_proc_max = pr;

        if (prof_report_ms == 0.0)
          prof_report_ms = now;
        else if (now - prof_report_ms >= 2000.0)
        {
          const double n = (double) prof_cycles;
          std::string line;
          for (auto& pit : prof_port)
          {
            const Op3ProfPort &pp = pit.second;
            const double tot = (double)(pp.ok + pp.fail);
            char buf[256];
            std::string tail = pit.first.substr(pit.first.find_last_of('/') + 1);
            snprintf(buf, sizeof(buf),
                     " | %.12s rx_ok=%.1f%% rx_avg=%.2f avail@rx=%ld avail@sw=%ld kosong=%.0f%% err=%d brtx[ok=%ld busy=%ld lain=%ld] swtx[ok=%ld busy=%ld lain=%ld]",
                     tail.c_str(), tot > 0 ? 100.0 * pp.ok / tot : 0.0,
                     tot > 0 ? pp.rx_sum / tot : 0.0,
                     tot > 0 ? (long)(pp.avail_sum / (long)tot) : 0L,
                     tot > 0 ? (long)(pp.avail_presw_sum / (long)tot) : 0L,
                     tot > 0 ? 100.0 * pp.avail_zero / tot : 0.0, pp.last_rx_result,
                     pp.tx_ok, pp.tx_busy, pp.tx_other,
                     pp.sw_ok, pp.sw_busy, pp.sw_other);
            line += buf;
          }
          for (auto& pit : prof_port_sensor)
          {
            const Op3ProfPort &pp = pit.second;
            const double tot = (double)(pp.ok + pp.fail);
            char buf[160];
            std::string tail = pit.first.substr(pit.first.find_last_of('/') + 1);
            snprintf(buf, sizeof(buf), " | %.12s(sensor) rx_ok=%.1f%% avail=%ld",
                     tail.c_str(), tot > 0 ? 100.0 * pp.ok / tot : 0.0,
                     tot > 0 ? (long)(pp.avail_sum / (long)tot) : 0L);
            line += buf;
          }
          RCLCPP_INFO(this->get_logger(),
                      "[PROF] siklus=%ld/dtk proc_avg=%.2f proc_max=%.2f mod_avg=%.2f mod_max=%.2f sw_avg=%.2f sw_max=%.2f jeda_avg=%.2f jeda_min=%.2f%s",
                      (long)(n / ((now - prof_report_ms) / 1000.0)),
                      prof_proc_sum / n, prof_proc_max,
                      prof_mod_sum / n, prof_mod_max,
                      prof_sw_sum / n, prof_sw_max,
                      prof_gap_sum / n, prof_gap_min, line.c_str());

          prof_report_ms = now;
          prof_cycles = 0;
          prof_gap_sum = 0.0; prof_gap_min = 1e9;
          prof_proc_sum = 0.0; prof_proc_max = 0.0;
          prof_mod_sum = 0.0; prof_mod_max = 0.0;
          prof_sw_sum = 0.0; prof_sw_max = 0.0;
          for (auto& pit : prof_port) pit.second = Op3ProfPort();
          for (auto& pit : prof_port_sensor) pit.second = Op3ProfPort();
        }
      }

      if (DEBUG_PRINT)
      {
        rclcpp::Duration time_duration = rclcpp::Clock().now() - start_time;
        fprintf(stderr, "(%2.6f) SyncWrite & BulkRead Tx \n", time_duration.nanoseconds() * 0.000001);
      }
    }
    else if (gazebo_mode_ == true)
    {
      std_msgs::msg::Float64 joint_msg;

      for (auto& dxl_it : robot_->dxls_)
      {
        std::string     joint_name  = dxl_it.first;
        Dynamixel      *dxl         = dxl_it.second;
        DynamixelState *dxl_state   = dxl_it.second->dxl_state_;
        
        if (dxl->ctrl_module_name_ == "none")
        {
          joint_msg.data = dxl_state->goal_position_;
          gazebo_joint_position_pub_[joint_name]->publish(joint_msg);
        }
      }

      for (auto module_it = motion_modules_.begin(); module_it != motion_modules_.end(); module_it++)
      {
        if ((*module_it)->getModuleEnable() == false)
          continue;

        for (auto& dxl_it : robot_->dxls_)
        {
          std::string     joint_name  = dxl_it.first;
          Dynamixel      *dxl         = dxl_it.second;
          DynamixelState *dxl_state   = dxl_it.second->dxl_state_;

          if (dxl->ctrl_module_name_ == (*module_it)->getModuleName())
          {
            if ((*module_it)->getControlMode() == PositionControl)
            {
              joint_msg.data = dxl_state->goal_position_;
              gazebo_joint_position_pub_[joint_name]->publish(joint_msg);
            }
            else if ((*module_it)->getControlMode() == VelocityControl)
            {
              joint_msg.data = dxl_state->goal_velocity_;
              gazebo_joint_velocity_pub_[joint_name]->publish(joint_msg);
            }
            else if ((*module_it)->getControlMode() == TorqueControl)
            {
              joint_msg.data = dxl_state->goal_torque_;
              gazebo_joint_effort_pub_[joint_name]->publish(joint_msg);
            }
          }
        }
      }
    }
  }
  else if (controller_mode_ == DirectControlMode)
  {
    if(gazebo_mode_ == false)
    {
      // Port sensor terpisah: Rx+Tx dituntaskan di awal siklus. Alasan lengkap
      // ada di cabang MotionModuleMode di atas (split transaction USB memakan
      // byte balasan servo kalau keduanya bertabrakan waktu).
      for (auto& it : port_to_bulk_read_)
      {
        if (sensor_only_ports.count(it.first) == 0 || do_sensor == false)
          continue;
        robot_->ports_[it.first]->setPacketTimeout(1.0);
        it.second->rxPacket();
        it.second->txPacket();
      }

      // BulkRead Rx
      for (auto& it : port_to_bulk_read_)
      {
        if (sensor_only_ports.count(it.first) != 0)
          continue;   // sudah diurus di awal siklus

        // Anggaran tunggu Rx. Nilainya KECIL dengan sengaja -- ini bukan sekadar
        // "berapa lama boleh menunggu", tapi penentu apakah loop ini stabil.
        //
        // Bulk read di sini dipipa: permintaan dikirim di AKHIR process(),
        // balasannya dibaca di AWAL process() berikutnya. Jadi waktu yang
        // dipunyai servo untuk menjawab = control_cycle - lama process(). Kalau
        // Rx gagal lalu menunggu lama, process() jadi panjang, jeda untuk
        // balasan BERIKUTNYA menyusut, dan kegagalan berikutnya jadi lebih
        // mungkin. Umpan balik positif: sekali tersandung, loop terkunci gagal.
        //
        // Terukur di robot ini (2026-08-25, OP3_PROFILE=1): dengan timeout 3 ms
        // manager mulai sehat (rx_ok 20%) lalu RUNTUH ke 0% dalam ~8 detik dan
        // tidak pernah pulih. Sebabnya persis di atas: 3 ms tunggu + 0,15 ms
        // hitung = jeda tinggal 4,87 ms, sedangkan balasan 20 servo baru
        // lengkap ~4,5 ms setelah write() (3,88 ms di kabel + antrean tulis
        // sync-write yang mendahuluinya). Sisa 0,4 ms -- terlalu tipis.
        //
        // Dengan 1 ms: jeda 6,85 ms lawan kebutuhan 4,5 ms. Kegagalan sesekali
        // tetap termaafkan, tapi tidak bisa lagi memakan jeda sampai terkunci.
        //
        // Kalau ini dinaikkan lagi, gerakan kepala akan patah-patah lagi.
        // Bukti tandingan yang penting: loop 125 Hz yang sama persis di luar ROS
        // (scripts/op3_bus_loop.py) berhasil 100% selama 20 detik -- bus, servo,
        // U2D2, dan latency_timer=1 semuanya SEHAT. Yang salah cuma anggaran
        // waktunya.
        robot_->ports_[it.first]->setPacketTimeout(1.0);
        it.second->rxPacket();
      }

      // -> save to robot->dxls_[]->dxl_state_
      if (robot_->dxls_.size() > 0)
      {
        for (auto& dxl_it : robot_->dxls_)
        {
          Dynamixel  *dxl         = dxl_it.second;
          std::string port_name   = dxl_it.second->port_name_;
          std::string joint_name  = dxl_it.first;

          if (dxl->bulk_read_items_.size() > 0)
          {
            uint32_t data = 0;
            for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
            {
              ControlTableItem *item = dxl->bulk_read_items_[i];
              if (port_to_bulk_read_[port_name]->isAvailable(dxl->id_, item->address_, item->data_length_) == true)
              {
                data = port_to_bulk_read_[port_name]->getData(dxl->id_, item->address_, item->data_length_);

                // change dxl_state
                if (dxl->present_position_item_ != 0 && item->item_name_ == dxl->present_position_item_->item_name_)
                {
                  dxl->dxl_state_->present_position_ = dxl->convertValue2Radian(data) - dxl->dxl_state_->position_offset_ * offset_ratio_;
                }
                else if (dxl->present_velocity_item_ != 0 && item->item_name_ == dxl->present_velocity_item_->item_name_)
                  dxl->dxl_state_->present_velocity_ = dxl->convertValue2Velocity(data);
                else if (dxl->present_current_item_ != 0 && item->item_name_ == dxl->present_current_item_->item_name_)
                  dxl->dxl_state_->present_torque_ = dxl->convertValue2Torque(data);
                else if (dxl->goal_position_item_ != 0 && item->item_name_ == dxl->goal_position_item_->item_name_)
                {
                  dxl->dxl_state_->goal_position_ = dxl->convertValue2Radian(data) - dxl->dxl_state_->position_offset_ * offset_ratio_;
                }
                else if (dxl->goal_velocity_item_ != 0 && item->item_name_ == dxl->goal_velocity_item_->item_name_)
                  dxl->dxl_state_->goal_velocity_ = dxl->convertValue2Velocity(data);
                else if (dxl->goal_current_item_ != 0 && item->item_name_ == dxl->goal_current_item_->item_name_)
                  dxl->dxl_state_->goal_torque_ = dxl->convertValue2Torque(data);

                dxl->dxl_state_->bulk_read_table_[item->item_name_] = data;
              }
            }

            // -> update time stamp to Robot->dxls[]->dynamixel_state.update_time_stamp
            dxl->dxl_state_->update_time_stamp_ = TimeStamp(present_state.header.stamp.sec, present_state.header.stamp.nanosec);
          }
        }
      }

      queue_mutex_.lock();

//      for (auto& it : port_to_sync_write_position_)
//      {
//        it.second->txPacket();
//        it.second->clearParam();
//      }

      if (direct_sync_write_.size() > 0)
      {
        for (int i = 0; i < direct_sync_write_.size(); i++)
        {
          direct_sync_write_[i]->txPacket();
          direct_sync_write_[i]->clearParam();
        }
        direct_sync_write_.clear();
        direct_sync_write_key_.clear();
      }

      queue_mutex_.unlock();

      // BulkRead Tx
      for (auto& it : port_to_bulk_read_)
      {
        if (sensor_only_ports.count(it.first) != 0)
          continue;   // sudah dikirim di awal siklus
        it.second->txPacket();
      }
    }
  }

  // Call SensorModule Process()
  // -> for loop : call SensorModule list -> Process()
  if (sensor_modules_.size() > 0)
  {
    for (auto module_it = sensor_modules_.begin(); module_it != sensor_modules_.end(); module_it++)
    {
      (*module_it)->process(robot_->dxls_, robot_->sensors_);

      for (auto& it : (*module_it)->result_)
        sensor_result_[it.first] = it.second;
    }
  }

  if (DEBUG_PRINT)
  {
    rclcpp::Duration time_duration = rclcpp::Clock().now() - start_time;
    fprintf(stderr, "(%2.6f) SensorModule Process() & save result \n", time_duration.nanoseconds() * 0.000001);
  }

  if (controller_mode_ == MotionModuleMode)
  {
    // Call MotionModule Process()
    // -> for loop : call MotionModule list -> Process()
    if (motion_modules_.size() > 0)
    {
      queue_mutex_.lock();

      for (auto module_it = motion_modules_.begin(); module_it != motion_modules_.end(); module_it++)
      {
        if ((*module_it)->getModuleEnable() == false)
          continue;

        (*module_it)->process(robot_->dxls_, sensor_result_);

        // for loop : joint list
        for (auto& dxl_it : robot_->dxls_)
        {
          std::string     joint_name  = dxl_it.first;
          Dynamixel      *dxl         = dxl_it.second;
          DynamixelState *dxl_state   = dxl_it.second->dxl_state_;

          if (dxl->ctrl_module_name_ == (*module_it)->getModuleName())
          {
            //do_sync_write = true;
            DynamixelState *result_state = (*module_it)->result_[joint_name];

            if (result_state == NULL)
            {
              RCLCPP_ERROR(this->get_logger(), "[%s] %s ", (*module_it)->getModuleName().c_str(), joint_name.c_str());
              continue;
            }

            // TODO: check update time stamp ?

            if ((*module_it)->getControlMode() == PositionControl)
            {
              dxl_state->goal_position_ = result_state->goal_position_;

              if (gazebo_mode_ == false)
              {
                // add offset
                uint32_t pos_data;
                pos_data= dxl->convertRadian2Value(dxl_state->goal_position_ + dxl_state->position_offset_ * offset_ratio_);

                uint8_t sync_write_data[4] = { 0 };
                sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(pos_data));
                sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(pos_data));
                sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(pos_data));
                sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(pos_data));

                if (port_to_sync_write_position_[dxl->port_name_] != NULL)
                  port_to_sync_write_position_[dxl->port_name_]->changeParam(dxl->id_, sync_write_data);

                // if position p gain value is changed -> sync write
                if (result_state->position_p_gain_ != NONE_GAIN && dxl_state->position_p_gain_ != result_state->position_p_gain_)
                {
                  dxl_state->position_p_gain_ = result_state->position_p_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->position_p_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->position_p_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->position_p_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->position_p_gain_));

                  if (port_to_sync_write_position_p_gain_[dxl->port_name_] != NULL)
                    port_to_sync_write_position_p_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }

                // if position i gain value is changed -> sync write
                if (result_state->position_i_gain_ != NONE_GAIN && dxl_state->position_i_gain_ != result_state->position_i_gain_)
                {
                  dxl_state->position_i_gain_ = result_state->position_i_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->position_i_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->position_i_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->position_i_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->position_i_gain_));

                  if (port_to_sync_write_position_i_gain_[dxl->port_name_] != NULL)
                    port_to_sync_write_position_i_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }

                // if position d gain value is changed -> sync write
                if (result_state->position_d_gain_ != NONE_GAIN && dxl_state->position_d_gain_ != result_state->position_d_gain_)
                {
                  dxl_state->position_d_gain_ = result_state->position_d_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->position_d_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->position_d_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->position_d_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->position_d_gain_));

                  if (port_to_sync_write_position_d_gain_[dxl->port_name_] != NULL)
                    port_to_sync_write_position_d_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }
                
                // if velocity p gain gain value is changed -> sync write
                if (result_state->velocity_p_gain_ != NONE_GAIN && dxl_state->velocity_p_gain_ != result_state->velocity_p_gain_)
                {
                  dxl_state->velocity_p_gain_ = result_state->velocity_p_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->velocity_p_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->velocity_p_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->velocity_p_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->velocity_p_gain_));

                  if (port_to_sync_write_velocity_p_gain_[dxl->port_name_] != NULL)
                    port_to_sync_write_velocity_p_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }

                // if velocity i gain value is changed -> sync write
                if (result_state->velocity_i_gain_ != NONE_GAIN && dxl_state->velocity_i_gain_ != result_state->velocity_i_gain_)
                {
                  dxl_state->velocity_i_gain_ = result_state->velocity_i_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->velocity_i_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->velocity_i_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->velocity_i_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->velocity_i_gain_));

                  if (port_to_sync_write_velocity_i_gain_[dxl->port_name_] != NULL)
                    port_to_sync_write_velocity_i_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }

              }
            }
            else if ((*module_it)->getControlMode() == VelocityControl)
            {
              dxl_state->goal_velocity_ = result_state->goal_velocity_;

              if (gazebo_mode_ == false)
              {
                uint32_t vel_data = dxl->convertVelocity2Value(dxl_state->goal_velocity_);
                uint8_t sync_write_data[4] = { 0 };
                sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(vel_data));
                sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(vel_data));
                sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(vel_data));
                sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(vel_data));

                if (port_to_sync_write_velocity_[dxl->port_name_] != NULL)
                  port_to_sync_write_velocity_[dxl->port_name_]->changeParam(dxl->id_, sync_write_data);

                // if velocity p gain gain value is changed -> sync write
                if (result_state->velocity_p_gain_ != NONE_GAIN && dxl_state->velocity_p_gain_ != result_state->velocity_p_gain_)
                {
                  dxl_state->velocity_p_gain_ = result_state->velocity_p_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->velocity_p_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->velocity_p_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->velocity_p_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->velocity_p_gain_));

                  if (port_to_sync_write_velocity_p_gain_[dxl->port_name_] != NULL)
                    port_to_sync_write_velocity_p_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }

                // if velocity i gain value is changed -> sync write
                if (result_state->velocity_i_gain_ != NONE_GAIN && dxl_state->velocity_i_gain_ != result_state->velocity_i_gain_)
                {
                  dxl_state->velocity_i_gain_ = result_state->velocity_i_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->velocity_i_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->velocity_i_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->velocity_i_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->velocity_i_gain_));

                  if (port_to_sync_write_velocity_i_gain_[dxl->port_name_] != NULL)
                    port_to_sync_write_velocity_i_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }

                // if velocity d gain value is changed -> sync write
                if (result_state->velocity_d_gain_ != NONE_GAIN && dxl_state->velocity_d_gain_ != result_state->velocity_d_gain_)
                {
                  dxl_state->velocity_d_gain_ = result_state->velocity_d_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->velocity_d_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->velocity_d_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->velocity_d_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->velocity_d_gain_));

                  if (port_to_sync_write_velocity_d_gain_[dxl->port_name_] != NULL)
                    port_to_sync_write_velocity_d_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }
              }
            }
            else if ((*module_it)->getControlMode() == TorqueControl)
            {
              dxl_state->goal_torque_ = result_state->goal_torque_;

              if (gazebo_mode_ == false)
              {
                uint32_t curr_data = dxl->convertTorque2Value(dxl_state->goal_torque_);
                uint8_t sync_write_data[2] = { 0 };
                sync_write_data[0] = DXL_LOBYTE(curr_data);
                sync_write_data[1] = DXL_HIBYTE(curr_data);

                if (port_to_sync_write_current_[dxl->port_name_] != NULL)
                  port_to_sync_write_current_[dxl->port_name_]->changeParam(dxl->id_, sync_write_data);
              }
            }
          }
        }
      }

      queue_mutex_.unlock();
    }

    if (DEBUG_PRINT)
    {
      rclcpp::Duration time_duration = rclcpp::Clock().now() - start_time;
      fprintf(stderr, "(%2.6f) MotionModule Process() & save result \n", time_duration.nanoseconds() * 0.000001);
    }
  }

  // publish present & goal position
  for (auto& dxl_it : robot_->dxls_)
  {
    std::string joint_name  = dxl_it.first;
    Dynamixel  *dxl         = dxl_it.second;

    present_state.name.push_back(joint_name);
    present_state.position.push_back(dxl->dxl_state_->present_position_);
    present_state.velocity.push_back(dxl->dxl_state_->present_velocity_);
    present_state.effort.push_back(dxl->dxl_state_->present_torque_);

    goal_state.name.push_back(joint_name);
    goal_state.position.push_back(dxl->dxl_state_->goal_position_);
    goal_state.velocity.push_back(dxl->dxl_state_->goal_velocity_);
    goal_state.effort.push_back(dxl->dxl_state_->goal_torque_);
  }

  // -> publish present joint_states & goal joint states topic
  present_joint_state_pub_->publish(present_state);
  goal_joint_state_pub_->publish(goal_state);

  if (DEBUG_PRINT)
  {
    rclcpp::Duration time_duration = rclcpp::Clock().now() - start_time;
    fprintf(stderr, "(%2.6f) Process() DONE \n", time_duration.nanoseconds() * 0.000001);
  }

  is_process_running = false;
}

void RobotisController::addMotionModule(MotionModule *module)
{
  // check whether the module name already exists
  for (auto m_it = motion_modules_.begin(); m_it != motion_modules_.end(); m_it++)
  {
    if ((*m_it)->getModuleName() == module->getModuleName())
    {
      RCLCPP_ERROR(this->get_logger(), "Motion Module Name [%s] already exist !!", module->getModuleName().c_str());
      return;
    }
  }

  module->initialize(robot_->getControlCycle(), robot_);
  motion_modules_.push_back(module);
  motion_modules_.unique();
}

void RobotisController::removeMotionModule(MotionModule *module)
{
  motion_modules_.remove(module);
}

void RobotisController::addSensorModule(SensorModule *module)
{
  // check whether the module name already exists
  for (auto m_it = sensor_modules_.begin(); m_it != sensor_modules_.end(); m_it++)
  {
    if ((*m_it)->getModuleName() == module->getModuleName())
    {
      RCLCPP_ERROR(this->get_logger(), "Sensor Module Name [%s] already exist !!", module->getModuleName().c_str());
      return;
    }
  }

  module->initialize(robot_->getControlCycle(), robot_);
  sensor_modules_.push_back(module);
  sensor_modules_.unique();
}

void RobotisController::removeSensorModule(SensorModule *module)
{
  sensor_modules_.remove(module);
}

void RobotisController::writeControlTableCallback(const robotis_controller_msgs::msg::WriteControlTable::SharedPtr msg)
{
  Device *device = NULL;

  if (DEBUG_PRINT)
    fprintf(stderr, "[WriteControlTable] led control msg received\n");

  auto dev_it1 = robot_->dxls_.find(msg->joint_name);
  if(dev_it1 != robot_->dxls_.end())
  {
    device = dev_it1->second;
  }
  else
  {
    auto dev_it2 = robot_->sensors_.find(msg->joint_name);
    if(dev_it2 != robot_->sensors_.end())
    {
      device = dev_it2->second;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "[WriteControlTable] Unknown device : %s", msg->joint_name.c_str());
      return;
    }
  }

  ControlTableItem *item = NULL;
  auto item_it = device->ctrl_table_.find(msg->start_item_name);
  if(item_it != device->ctrl_table_.end())
  {
    item = item_it->second;
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "[WriteControlTable] Unknown item : %s", msg->start_item_name.c_str());
    return;
  }

  dynamixel::PortHandler   *port           = robot_->ports_[device->port_name_];
  dynamixel::PacketHandler *packet_handler = dynamixel::PacketHandler::getPacketHandler(device->protocol_version_);

  if (item->access_type_ == Read)
    return;

  queue_mutex_.lock();

  direct_sync_write_.push_back(new dynamixel::GroupSyncWrite(port, packet_handler, item->address_, msg->data_length));
  direct_sync_write_key_.push_back(std::make_pair(item->address_, (uint16_t) msg->data_length));
  direct_sync_write_[direct_sync_write_.size() - 1]->addParam(device->id_, (uint8_t *)(msg->data.data()));
  
//  fprintf(stderr, "[WriteControlTable] %s -> %s : ", msg->joint_name.c_str(), msg->start_item_name.c_str());
//  for (auto &dt : msg->data)
//	  fprintf(stderr, "%02X ", dt);
//  fprintf(stderr, "\n");

  queue_mutex_.unlock();

}

void RobotisController::syncWriteItemCallback(const robotis_controller_msgs::msg::SyncWriteItem::SharedPtr msg)
{
  for (int i = 0; i < msg->joint_name.size(); i++)
  {
    Device *device;

    auto d_it1 = robot_->dxls_.find(msg->joint_name[i]);
    if (d_it1 != robot_->dxls_.end())
    {
      device = d_it1->second;
    }
    else
    {
      auto d_it2 = robot_->sensors_.find(msg->joint_name[i]);
      if (d_it2 != robot_->sensors_.end())
      {
        device = d_it2->second;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "[SyncWriteItem] Unknown device : %s", msg->joint_name[i].c_str());
        continue;
      }
    }

    ControlTableItem *item = NULL;
    auto item_it = device->ctrl_table_.find(msg->item_name);
    if (item_it != device->ctrl_table_.end())
    {
      item = item_it->second;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "[SyncWriteItem] Unknown item : %s", msg->item_name.c_str());
      continue;
    }

    dynamixel::PortHandler *port = robot_->ports_[device->port_name_];
    dynamixel::PacketHandler *packet_handler = dynamixel::PacketHandler::getPacketHandler(device->protocol_version_);

    if (item->access_type_ == Read)
      continue;

    queue_mutex_.lock();

    // Reuse an existing packet only when it targets the SAME register. Matching
    // on the port alone meant that two different items queued inside one control
    // cycle -- the studio sends profile_velocity, profile_acceleration and
    // torque_enable back to back -- shared one GroupSyncWrite, and the later
    // items were written to the first item's address with the first item's data
    // length. Torque commands then landed in profile_velocity: the button did
    // nothing and the joint's speed got scrambled instead, intermittently,
    // depending on which writes happened to fall in the same cycle.
    int idx = 0;
    for (idx = 0; idx < direct_sync_write_.size(); idx++)
    {
      if (direct_sync_write_[idx]->getPortHandler() == port
          && direct_sync_write_[idx]->getPacketHandler() == packet_handler
          && direct_sync_write_key_[idx].first == item->address_
          && direct_sync_write_key_[idx].second == item->data_length_)
        break;
    }

    if (idx == direct_sync_write_.size())
    {
      direct_sync_write_.push_back(new dynamixel::GroupSyncWrite(port, packet_handler, item->address_, item->data_length_));
      direct_sync_write_key_.push_back(std::make_pair(item->address_, item->data_length_));
    }

    uint8_t *data = new uint8_t[item->data_length_];
    if (item->data_length_ == 1)
      data[0] = (uint8_t)msg->value[i];
    else if (item->data_length_ == 2)
    {
      data[0] = DXL_LOBYTE((uint16_t)msg->value[i]);
      data[1] = DXL_HIBYTE((uint16_t)msg->value[i]);
    }
    else if (item->data_length_ == 4)
    {
      data[0] = DXL_LOBYTE(DXL_LOWORD((uint32_t)msg->value[i]));
      data[1] = DXL_HIBYTE(DXL_LOWORD((uint32_t)msg->value[i]));
      data[2] = DXL_LOBYTE(DXL_HIWORD((uint32_t)msg->value[i]));
      data[3] = DXL_HIBYTE(DXL_HIWORD((uint32_t)msg->value[i]));
    }
    direct_sync_write_[idx]->addParam(device->id_, data);
    delete[] data;

    queue_mutex_.unlock();
  }
}

void RobotisController::setControllerModeCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "DirectControlMode")
  {
    for (auto& it : port_to_bulk_read_)
    {
      robot_->ports_[it.first]->setPacketTimeout(0.0);
      it.second->rxPacket();
    }
    controller_mode_ = DirectControlMode;
  }
  else if (msg->data == "MotionModuleMode")
  {
    for (auto& it : port_to_bulk_read_)
    {
      it.second->txPacket();
    }
    controller_mode_ = MotionModuleMode;
  }
}

void RobotisController::setJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  queue_mutex_.lock();

  for (int i = 0; i < msg->name.size(); i++)
  {
    Dynamixel *dxl = robot_->dxls_[msg->name[i]];
    if (dxl == NULL)
      continue;

    if ((controller_mode_ == DirectControlMode) || 
        (controller_mode_ == MotionModuleMode && dxl->ctrl_module_name_ == "none"))
    {
      dxl->dxl_state_->goal_position_ = (double)msg->position[i];
      
      if (gazebo_mode_ == false)
      {
        // add offset
        uint32_t pos_data;
        pos_data = dxl->convertRadian2Value(dxl->dxl_state_->goal_position_ + dxl->dxl_state_->position_offset_ * offset_ratio_);

        uint8_t sync_write_data[4] = { 0 };
        sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(pos_data));
        sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(pos_data));
        sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(pos_data));
        sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(pos_data));
        
        if (port_to_sync_write_position_[dxl->port_name_] != NULL)
          port_to_sync_write_position_[dxl->port_name_]->changeParam(dxl->id_, sync_write_data);
      }
    }
  }

  queue_mutex_.unlock();
}

void RobotisController::setCtrlModuleCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if(set_module_thread_.joinable())
    set_module_thread_.join();

  std::string _module_name_to_set = msg->data;

  set_module_thread_ = std::thread(&RobotisController::setCtrlModuleThread, this, _module_name_to_set);
}

void RobotisController::setCtrlModule(std::string module_name)
{
  if(set_module_thread_.joinable())
    set_module_thread_.join();

  set_module_thread_ = std::thread(&RobotisController::setCtrlModuleThread, this, module_name);
}

void RobotisController::setJointCtrlModuleCallback(robotis_controller_msgs::msg::JointCtrlModule::SharedPtr msg)
{
  if (msg->joint_name.size() != msg->module_name.size())
    return;

  if(set_module_thread_.joinable())
    set_module_thread_.join();

  set_module_thread_ = std::thread(&RobotisController::setJointCtrlModuleThread, this, msg);
}

void RobotisController::enableOffsetCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  is_offset_enabled_ = (bool)msg->data;
  if(is_offset_enabled_)
    offset_ratio_ = 0.0;
  else
    offset_ratio_ = 1.0;  
}

bool RobotisController::getJointCtrlModuleService(const std::shared_ptr<robotis_controller_msgs::srv::GetJointModule::Request> req,
    std::shared_ptr<robotis_controller_msgs::srv::GetJointModule::Response> res)
{
  for (unsigned int idx = 0; idx < req->joint_name.size(); idx++)
  {
    auto d_it = robot_->dxls_.find((std::string)(req->joint_name[idx]));
    if (d_it != robot_->dxls_.end())
    {
      res->joint_name.push_back(req->joint_name[idx]);
      res->module_name.push_back(d_it->second->ctrl_module_name_);
    }
  }

  if (res->joint_name.size() == 0)
    return false;

  return true;
}

bool RobotisController::setJointCtrlModuleService(const std::shared_ptr<robotis_controller_msgs::srv::SetJointModule::Request> req,
                          std::shared_ptr<robotis_controller_msgs::srv::SetJointModule::Response> res)
{
  if(set_module_thread_.joinable())
    set_module_thread_.join();

  robotis_controller_msgs::msg::JointCtrlModule modules;
  modules.joint_name = req->joint_name;
  modules.module_name = req->module_name;

  auto msg_ptr = std::make_shared<robotis_controller_msgs::msg::JointCtrlModule>(modules);

  if (modules.joint_name.size() != modules.module_name.size())
    return false;

  set_module_thread_ = std::thread(&RobotisController::setJointCtrlModuleThread, this, msg_ptr);

  set_module_thread_.join();

  res->result = true;
  return true;
}

bool RobotisController::setCtrlModuleService(const std::shared_ptr<robotis_controller_msgs::srv::SetModule::Request> req,
                       std::shared_ptr<robotis_controller_msgs::srv::SetModule::Response> res)
{
  if(set_module_thread_.joinable())
    set_module_thread_.join();

  std::string _module_name_to_set = req->module_name;

  try
  {
    set_module_thread_ = std::thread(&RobotisController::setCtrlModuleThread, this, _module_name_to_set);

    // 스레드가 성공적으로 종료될 때까지 대기
    set_module_thread_.join();

    // 서비스 응답 설정
    res->result = true;
    RCLCPP_INFO(this->get_logger(), "Successfully set control module: %s", _module_name_to_set.c_str());
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error while setting control module: %s", e.what());
    res->result = false;
    return false;
  }
  catch (...)
  {
    RCLCPP_ERROR(this->get_logger(), "Unknown error occurred while setting control module.");
    res->result = false;
    return false;
  }

  return true;
}

bool RobotisController::loadOffsetService(const std::shared_ptr<robotis_controller_msgs::srv::LoadOffset::Request> req,
                      std::shared_ptr<robotis_controller_msgs::srv::LoadOffset::Response> res)
{
  loadOffset((std::string)req->file_path);
  res->result = true;
  return true;
}

void RobotisController::setJointCtrlModuleThread(robotis_controller_msgs::msg::JointCtrlModule::SharedPtr msg)
{
  // Siapa mengambil sendi apa. Kejadiannya jarang (hanya saat modul berpindah)
  // tapi tanpa catatan ini "tiba-tiba parameternya lain" di tengah demo mustahil
  // dilacak: permintaan lewat service tidak muncul di topik mana pun.
  {
    std::string ringkas;
    for (unsigned int idx = 0; idx < msg->joint_name.size() && idx < msg->module_name.size(); idx++)
    {
      if (idx != 0)
        ringkas += ", ";
      ringkas += msg->joint_name[idx] + "=" + msg->module_name[idx];
    }
    RCLCPP_WARN(this->get_logger(), "SET MODUL per-sendi: %s", ringkas.c_str());
  }

  // stop module list
  std::list<MotionModule *> _stop_modules;
  std::list<MotionModule *> _enable_modules;

  for(unsigned int idx = 0; idx < msg->joint_name.size(); idx++)
  {
    Dynamixel *_dxl = nullptr;
    auto _dxl_it = robot_->dxls_.find((msg->joint_name[idx]));
    if(_dxl_it != robot_->dxls_.end())
      _dxl = _dxl_it->second;
    else
      continue;

    // enqueue
    if(_dxl->ctrl_module_name_ != msg->module_name[idx])
    {
      for(auto _stop_m_it = motion_modules_.begin(); _stop_m_it != motion_modules_.end(); _stop_m_it++)
      {
        if((*_stop_m_it)->getModuleName() == _dxl->ctrl_module_name_ && (*_stop_m_it)->getModuleEnable() == true)
          _stop_modules.push_back(*_stop_m_it);
      }
    }
  }

  // stop the module
  _stop_modules.unique();
  for(auto _stop_m_it = _stop_modules.begin(); _stop_m_it != _stop_modules.end(); _stop_m_it++)
  {
    (*_stop_m_it)->stop();
  }

  // wait to stop
  for(auto _stop_m_it = _stop_modules.begin(); _stop_m_it != _stop_modules.end(); _stop_m_it++)
  {
    while((*_stop_m_it)->isRunning())
      usleep(robot_->getControlCycle() * 1000);
  }

  // disable module(s)
  for(auto _stop_m_it = _stop_modules.begin(); _stop_m_it != _stop_modules.end(); _stop_m_it++)
  {
    (*_stop_m_it)->setModuleEnable(false);
  }

  // set ctrl module
  queue_mutex_.lock();

  for(unsigned int idx = 0; idx < msg->joint_name.size(); idx++)
  {
    const std::string& ctrl_module = msg->module_name[idx];
    const std::string& joint_name = msg->joint_name[idx];

    Dynamixel *_dxl = nullptr;
    auto _dxl_it = robot_->dxls_.find(joint_name);
    if(_dxl_it != robot_->dxls_.end())
      _dxl = _dxl_it->second;
    else
      continue;

    // none
    if(ctrl_module == "" || ctrl_module == "none")
    {
      _dxl->ctrl_module_name_ = "none";

      if(gazebo_mode_ == true)
        continue;

      uint32_t _pos_data = _dxl->convertRadian2Value(_dxl->dxl_state_->goal_position_ + _dxl->dxl_state_->position_offset_ * offset_ratio_);

      uint8_t _sync_write_data[4];
      _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_pos_data));
      _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_pos_data));
      _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_pos_data));
      _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_pos_data));

      if(port_to_sync_write_position_[_dxl->port_name_] != nullptr)
        port_to_sync_write_position_[_dxl->port_name_]->addParam(_dxl->id_, _sync_write_data);

      if(port_to_sync_write_current_[_dxl->port_name_] != nullptr)
        port_to_sync_write_current_[_dxl->port_name_]->removeParam(_dxl->id_);
      if(port_to_sync_write_velocity_[_dxl->port_name_] != nullptr)
        port_to_sync_write_velocity_[_dxl->port_name_]->removeParam(_dxl->id_);
    }
    else
    {
      // check whether the module exist
      for(auto _m_it = motion_modules_.begin(); _m_it != motion_modules_.end(); _m_it++)
      {
        // if it exist
        if((*_m_it)->getModuleName() == ctrl_module)
        {
          auto _result_it = (*_m_it)->result_.find(joint_name);
          if(_result_it == (*_m_it)->result_.end())
            break;

          _dxl->ctrl_module_name_ = ctrl_module;

          // enqueue enable module list
          _enable_modules.push_back(*_m_it);
          ControlMode _mode = (*_m_it)->getControlMode();

          if(gazebo_mode_ == true)
            break;

          if(_mode == PositionControl)
          {
            uint32_t _pos_data = _dxl->convertRadian2Value(_dxl->dxl_state_->goal_position_ + _dxl->dxl_state_->position_offset_ * offset_ratio_);

            uint8_t _sync_write_data[4];
            _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_pos_data));
            _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_pos_data));
            _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_pos_data));
            _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_pos_data));

            if(port_to_sync_write_position_[_dxl->port_name_] != nullptr)
              port_to_sync_write_position_[_dxl->port_name_]->addParam(_dxl->id_, _sync_write_data);

            if(port_to_sync_write_current_[_dxl->port_name_] != nullptr)
              port_to_sync_write_current_[_dxl->port_name_]->removeParam(_dxl->id_);
            if(port_to_sync_write_velocity_[_dxl->port_name_] != nullptr)
              port_to_sync_write_velocity_[_dxl->port_name_]->removeParam(_dxl->id_);
          }
          else if(_mode == VelocityControl)
          {
            uint32_t _vel_data = _dxl->convertVelocity2Value(_dxl->dxl_state_->goal_velocity_);
            uint8_t _sync_write_data[4];
            _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_vel_data));
            _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_vel_data));
            _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_vel_data));
            _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_vel_data));

            if(port_to_sync_write_velocity_[_dxl->port_name_] != nullptr)
              port_to_sync_write_velocity_[_dxl->port_name_]->addParam(_dxl->id_, _sync_write_data);

            if(port_to_sync_write_current_[_dxl->port_name_] != nullptr)
              port_to_sync_write_current_[_dxl->port_name_]->removeParam(_dxl->id_);
            if(port_to_sync_write_position_[_dxl->port_name_] != nullptr)
              port_to_sync_write_position_[_dxl->port_name_]->removeParam(_dxl->id_);
          }
          else if(_mode == TorqueControl)
          {
            uint32_t _curr_data = _dxl->convertTorque2Value(_dxl->dxl_state_->goal_torque_);
            uint8_t _sync_write_data[4];
            _sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_curr_data));
            _sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_curr_data));
            _sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_curr_data));
            _sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_curr_data));

            if(port_to_sync_write_current_[_dxl->port_name_] != nullptr)
              port_to_sync_write_current_[_dxl->port_name_]->addParam(_dxl->id_, _sync_write_data);

            if(port_to_sync_write_velocity_[_dxl->port_name_] != nullptr)
              port_to_sync_write_velocity_[_dxl->port_name_]->removeParam(_dxl->id_);
            if(port_to_sync_write_position_[_dxl->port_name_] != nullptr)
              port_to_sync_write_position_[_dxl->port_name_]->removeParam(_dxl->id_);
          }
          break;
        }
      }
    }
  }

  // enable module(s)
  _enable_modules.unique();
  for(auto _m_it = _enable_modules.begin(); _m_it != _enable_modules.end(); _m_it++)
  {
    (*_m_it)->setModuleEnable(true);
  }

  // TODO: set indirect address
  // -> check module's control_mode

  queue_mutex_.unlock();

  // publish current module
  robotis_controller_msgs::msg::JointCtrlModule _current_module_msg;
  for(auto _dxl_iter = robot_->dxls_.begin(); _dxl_iter  != robot_->dxls_.end(); ++_dxl_iter)
  {
    _current_module_msg.joint_name.push_back(_dxl_iter->first);
    _current_module_msg.module_name.push_back(_dxl_iter->second->ctrl_module_name_);
  }

  if(_current_module_msg.joint_name.size() == _current_module_msg.module_name.size())
    current_module_pub_->publish(_current_module_msg);
}

void RobotisController::setCtrlModuleThread(std::string ctrl_module)
{
  // Jalur "satu modul mengambil semua sendi miliknya". Dicatat karena inilah
  // yang menyapu kepala/kaki secara tidak sengaja -- lihat catatan di atas.
  RCLCPP_WARN(this->get_logger(), "SET MODUL seluruh-modul: '%s'", ctrl_module.c_str());

  // stop module
  std::list<MotionModule *> stop_modules;

  if (ctrl_module == "" || ctrl_module == "none")
  {
    // enqueue all modules in order to stop
    for (auto m_it = motion_modules_.begin(); m_it != motion_modules_.end(); m_it++)
    {
      if ((*m_it)->getModuleEnable() == true)
        stop_modules.push_back(*m_it);
    }
  }
  else
  {
    for (auto m_it = motion_modules_.begin(); m_it != motion_modules_.end(); m_it++)
    {
      // if it exist
      if ((*m_it)->getModuleName() == ctrl_module)
      {
        // enqueue the module which lost control of joint in order to stop
        for (auto& result_it : (*m_it)->result_)
        {
          auto d_it = robot_->dxls_.find(result_it.first);

          if (d_it != robot_->dxls_.end())
          {
            // enqueue
            if (d_it->second->ctrl_module_name_ != ctrl_module)
            {
              for (auto stop_m_it = motion_modules_.begin(); stop_m_it != motion_modules_.end(); stop_m_it++)
              {
                if (((*stop_m_it)->getModuleName() == d_it->second->ctrl_module_name_) &&
                    ((*stop_m_it)->getModuleEnable() == true))
                {
                  stop_modules.push_back(*stop_m_it);
                }
              }
            }
          }
        }

        break;
      }
    }
  }

  // stop the module
  stop_modules.unique();
  for (auto stop_m_it = stop_modules.begin(); stop_m_it != stop_modules.end(); stop_m_it++)
  {
    (*stop_m_it)->stop();
  }

  // wait to stop
  for (auto stop_m_it = stop_modules.begin(); stop_m_it != stop_modules.end(); stop_m_it++)
  {
    while ((*stop_m_it)->isRunning())
      usleep(robot_->getControlCycle() * 1000);
  }

  // disable module(s)
  for(std::list<MotionModule *>::iterator _stop_m_it = stop_modules.begin(); _stop_m_it != stop_modules.end(); _stop_m_it++)
  {
    (*_stop_m_it)->setModuleEnable(false);
  }


  // set ctrl module
  queue_mutex_.lock();

  if (DEBUG_PRINT)
    RCLCPP_INFO(this->get_logger(), "set module : %s", ctrl_module.c_str());

  // none
  if ((ctrl_module == "") || (ctrl_module == "none"))
  {
    // set dxl's control module to "none"
    for (auto& d_it : robot_->dxls_)
    {
      Dynamixel *dxl = d_it.second;
      dxl->ctrl_module_name_ = "none";

      if (gazebo_mode_ == true)
        continue;

      uint32_t pos_data;
      pos_data = dxl->convertRadian2Value(dxl->dxl_state_->goal_position_ + dxl->dxl_state_->position_offset_ * offset_ratio_);

      uint8_t sync_write_data[4] = { 0 };
      sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(pos_data));
      sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(pos_data));
      sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(pos_data));
      sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(pos_data));

      if (port_to_sync_write_position_[dxl->port_name_] != NULL)
        port_to_sync_write_position_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

      if (port_to_sync_write_current_[dxl->port_name_] != NULL)
        port_to_sync_write_current_[dxl->port_name_]->removeParam(dxl->id_);
      if (port_to_sync_write_velocity_[dxl->port_name_] != NULL)
        port_to_sync_write_velocity_[dxl->port_name_]->removeParam(dxl->id_);
    }
  }
  else
  {
    // check whether the module exist
    for (auto m_it = motion_modules_.begin(); m_it != motion_modules_.end(); m_it++)
    {
      // if it exist
      if ((*m_it)->getModuleName() == ctrl_module)
      {
        ControlMode mode = (*m_it)->getControlMode();
        for (auto& result_it : (*m_it)->result_)
        {
          auto d_it = robot_->dxls_.find(result_it.first);
          if (d_it != robot_->dxls_.end())
          {
            Dynamixel *dxl = d_it->second;
            dxl->ctrl_module_name_ = ctrl_module;

            if (gazebo_mode_ == true)
              continue;

            if (mode == PositionControl)
            {
              uint32_t pos_data;
              pos_data = dxl->convertRadian2Value(dxl->dxl_state_->goal_position_ + dxl->dxl_state_->position_offset_ * offset_ratio_);

              uint8_t sync_write_data[4] = { 0 };
              sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(pos_data));
              sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(pos_data));
              sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(pos_data));
              sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(pos_data));

              if (port_to_sync_write_position_[dxl->port_name_] != NULL)
                port_to_sync_write_position_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

              if (port_to_sync_write_current_[dxl->port_name_] != NULL)
                port_to_sync_write_current_[dxl->port_name_]->removeParam(dxl->id_);
              if (port_to_sync_write_velocity_[dxl->port_name_] != NULL)
                port_to_sync_write_velocity_[dxl->port_name_]->removeParam(dxl->id_);
            }
            else if (mode == VelocityControl)
            {
              uint32_t vel_data = dxl->convertVelocity2Value(dxl->dxl_state_->goal_velocity_);
              uint8_t sync_write_data[4] = { 0 };
              sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(vel_data));
              sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(vel_data));
              sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(vel_data));
              sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(vel_data));

              if (port_to_sync_write_velocity_[dxl->port_name_] != NULL)
                port_to_sync_write_velocity_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

              if (port_to_sync_write_current_[dxl->port_name_] != NULL)
                port_to_sync_write_current_[dxl->port_name_]->removeParam(dxl->id_);
              if (port_to_sync_write_position_[dxl->port_name_] != NULL)
                port_to_sync_write_position_[dxl->port_name_]->removeParam(dxl->id_);
            }
            else if (mode == TorqueControl)
            {
              uint32_t curr_data = dxl->convertTorque2Value(dxl->dxl_state_->goal_torque_);
              uint8_t sync_write_data[4] = { 0 };
              sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(curr_data));
              sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(curr_data));
              sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(curr_data));
              sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(curr_data));

              if (port_to_sync_write_current_[dxl->port_name_] != NULL)
                port_to_sync_write_current_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

              if (port_to_sync_write_velocity_[dxl->port_name_] != NULL)
                port_to_sync_write_velocity_[dxl->port_name_]->removeParam(dxl->id_);
              if (port_to_sync_write_position_[dxl->port_name_] != NULL)
                port_to_sync_write_position_[dxl->port_name_]->removeParam(dxl->id_);
            }
          }
        }

        break;
      }
    }
  }

  for (auto m_it = motion_modules_.begin(); m_it != motion_modules_.end(); m_it++)
  {
    // set all used modules -> enable
    for (auto& d_it : robot_->dxls_)
    {
      if (d_it.second->ctrl_module_name_ == (*m_it)->getModuleName())
      {
        (*m_it)->setModuleEnable(true);
        break;
      }
    }
  }

  // TODO: set indirect address
  // -> check module's control_mode

  queue_mutex_.unlock();

  // publish current module
  robotis_controller_msgs::msg::JointCtrlModule current_module_msg;
  for (auto& dxl_iter : robot_->dxls_)
  {
    current_module_msg.joint_name.push_back(dxl_iter.first);
    current_module_msg.module_name.push_back(dxl_iter.second->ctrl_module_name_);
  }

  if (current_module_msg.joint_name.size() == current_module_msg.module_name.size())
    current_module_pub_->publish(current_module_msg);
}

void RobotisController::gazeboJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  queue_mutex_.lock();

  for (unsigned int i = 0; i < msg->name.size(); i++)
  {
    auto d_it = robot_->dxls_.find((std::string) msg->name[i]);
    if (d_it != robot_->dxls_.end())
    {
      d_it->second->dxl_state_->present_position_ = msg->position[i];
      d_it->second->dxl_state_->present_velocity_ = msg->velocity[i];
      d_it->second->dxl_state_->present_torque_ = msg->effort[i];
    }
  }

  if (init_pose_loaded_ == false)
  {
    for (auto& it : robot_->dxls_)
      it.second->dxl_state_->goal_position_ = it.second->dxl_state_->present_position_;
    init_pose_loaded_ = true;
  }

  queue_mutex_.unlock();
}

bool RobotisController::isTimerStopped()
{
  if (this->is_timer_running_)
  {
    if (DEBUG_PRINT == true)
      RCLCPP_WARN(this->get_logger(), "Process Timer is running.. STOP the timer first.");
    return false;
  }
  return true;
}

int RobotisController::ping(const std::string joint_name, uint8_t *error)
{
  return ping(joint_name, 0, error);
}
int RobotisController::ping(const std::string joint_name, uint16_t* model_number, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->ping(port_handler, dxl->id_, model_number, error);
}

int RobotisController::action(const std::string joint_name)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->action(port_handler, dxl->id_);
}
int RobotisController::reboot(const std::string joint_name, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->reboot(port_handler, dxl->id_, error);
}
int RobotisController::factoryReset(const std::string joint_name, uint8_t option, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->factoryReset(port_handler, dxl->id_, option, error);
}

int RobotisController::read(const std::string joint_name, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->readTxRx(port_handler, dxl->id_, address, length, data, error);
}

int RobotisController::readCtrlItem(const std::string joint_name, const std::string item_name, uint32_t *data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  ControlTableItem *item = dxl->ctrl_table_[item_name];
  if (item == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  int result = COMM_NOT_AVAILABLE;
  switch (item->data_length_)
  {
  case 1:
  {
    uint8_t read_data = 0;
    result = pkt_handler->read1ByteTxRx(port_handler, dxl->id_, item->address_, &read_data, error);
    if (result == COMM_SUCCESS)
      *data = read_data;
    break;
  }
  case 2:
  {
    uint16_t read_data = 0;
    result = pkt_handler->read2ByteTxRx(port_handler, dxl->id_, item->address_, &read_data, error);
    if (result == COMM_SUCCESS)
      *data = read_data;
    break;
  }
  case 4:
  {
    uint32_t read_data = 0;
    result = pkt_handler->read4ByteTxRx(port_handler, dxl->id_, item->address_, &read_data, error);
    if (result == COMM_SUCCESS)
      *data = read_data;
    break;
  }
  default:
    break;
  }
  return result;
}

int RobotisController::read1Byte(const std::string joint_name, uint16_t address, uint8_t *data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->read1ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::read2Byte(const std::string joint_name, uint16_t address, uint16_t *data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->read2ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::read4Byte(const std::string joint_name, uint16_t address, uint32_t *data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->read4ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::write(const std::string joint_name, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->writeTxRx(port_handler, dxl->id_, address, length, data, error);
}

int RobotisController::writeCtrlItem(const std::string joint_name, const std::string item_name, uint32_t data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  ControlTableItem *item = dxl->ctrl_table_[item_name];
  if (item == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  int       result      = COMM_NOT_AVAILABLE;
  uint8_t  *write_data  = new uint8_t[item->data_length_];
  if (item->data_length_ == 1)
  {
    write_data[0] = (uint8_t) data;
    result = pkt_handler->write1ByteTxRx(port_handler, dxl->id_, item->address_, data, error);
  }
  else if (item->data_length_ == 2)
  {
    write_data[0] = DXL_LOBYTE((uint16_t )data);
    write_data[1] = DXL_HIBYTE((uint16_t )data);
    result = pkt_handler->write2ByteTxRx(port_handler, dxl->id_, item->address_, data, error);
  }
  else if (item->data_length_ == 4)
  {
    write_data[0] = DXL_LOBYTE(DXL_LOWORD((uint32_t)data));
    write_data[1] = DXL_HIBYTE(DXL_LOWORD((uint32_t)data));
    write_data[2] = DXL_LOBYTE(DXL_HIWORD((uint32_t)data));
    write_data[3] = DXL_HIBYTE(DXL_HIWORD((uint32_t)data));
    result = pkt_handler->write4ByteTxRx(port_handler, dxl->id_, item->address_, data, error);
  }
  delete[] write_data;
  return result;
}

int RobotisController::write1Byte(const std::string joint_name, uint16_t address, uint8_t data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->write1ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::write2Byte(const std::string joint_name, uint16_t address, uint16_t data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->write2ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::write4Byte(const std::string joint_name, uint16_t address, uint32_t data, uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->write4ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::regWrite(const std::string joint_name, uint16_t address, uint16_t length, uint8_t *data,
    uint8_t *error)
{
  if (isTimerStopped() == false)
    return COMM_PORT_BUSY;

  Dynamixel *dxl = robot_->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler *pkt_handler   = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler   *port_handler  = robot_->ports_[dxl->port_name_];

  return pkt_handler->regWriteTxRx(port_handler, dxl->id_, address, length, data, error);
}
