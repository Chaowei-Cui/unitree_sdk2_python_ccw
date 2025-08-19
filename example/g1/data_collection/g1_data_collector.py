import time
import sys
import h5py
import os
from datetime import datetime
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

class G1DataCollector:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.002  # [2ms]
        self.duration_ = 0.0      # 数据收集持续时间
        self.counter_ = 0
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  
        self.low_state = None 
        self.update_mode_machine_ = False
        self.crc = CRC()
        
        # 数据存储
        self.data_buffer = []
        self.start_time = None
        self.data_dir = "g1_data_collection"
        
        # 创建数据目录
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)

    def Init(self):
        """初始化机器人连接和数据收集"""
        print("正在初始化机器人连接...")
        
        # 初始化运动切换器
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        # 检查并释放模式
        status, result = self.msc.CheckMode()
        while result['name']:
            print(f"释放模式: {result['name']}")
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)

        # 创建发布者
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

        # 创建订阅者
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)
        
        print("机器人连接初始化完成!")

    def Start(self, duration=10.0):
        """开始数据收集"""
        self.duration_ = duration
        self.start_time = time.time()
        print(f"开始数据收集，持续时间: {duration} 秒")
        
        # 创建控制线程
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        
        # 等待模式就绪
        while not self.update_mode_machine_:
            time.sleep(0.1)
            print("等待机器人状态...")

        if self.update_mode_machine_:
            print("机器人状态就绪，开始数据收集...")
            self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        """处理机器人状态数据"""
        self.low_state = msg

        if not self.update_mode_machine_:
            self.mode_machine_ = self.low_state.mode_machine
            self.update_mode_machine_ = True
        
        # 收集数据
        self.CollectData()
        
        self.counter_ += 1
        if self.counter_ % 500 == 0:
            elapsed = time.time() - self.start_time if self.start_time else 0
            print(f"已收集 {len(self.data_buffer)} 条数据，运行时间: {elapsed:.1f}s")

    def CollectData(self):
        """收集所有可用的传感器数据"""
        if not self.low_state:
            return
            
        current_time = time.time()
        elapsed_time = current_time - self.start_time if self.start_time else 0
        
        # 创建数据记录
        data_record = {
            'timestamp': current_time,
            'elapsed_time': elapsed_time,
            'counter': self.counter_
        }
        
        # IMU数据
        if hasattr(self.low_state, 'imu_state') and self.low_state.imu_state:
            imu = self.low_state.imu_state
            data_record.update({
                'imu_rpy_x': imu.rpy[0] if hasattr(imu, 'rpy') and len(imu.rpy) > 0 else 0,
                'imu_rpy_y': imu.rpy[1] if hasattr(imu, 'rpy') and len(imu.rpy) > 1 else 0,
                'imu_rpy_z': imu.rpy[2] if hasattr(imu, 'rpy') and len(imu.rpy) > 2 else 0,
                'imu_gyro_x': imu.gyro[0] if hasattr(imu, 'gyro') and len(imu.gyro) > 0 else 0,
                'imu_gyro_y': imu.gyro[1] if hasattr(imu, 'gyro') and len(imu.gyro) > 1 else 0,
                'imu_gyro_z': imu.gyro[2] if hasattr(imu, 'gyro') and len(imu.gyro) > 2 else 0,
                'imu_accel_x': imu.accel[0] if hasattr(imu, 'accel') and len(imu.accel) > 0 else 0,
                'imu_accel_y': imu.accel[1] if hasattr(imu, 'accel') and len(imu.accel) > 1 else 0,
                'imu_accel_z': imu.accel[2] if hasattr(imu, 'accel') and len(imu.accel) > 2 else 0,
                'imu_quaternion_w': imu.quaternion[0] if hasattr(imu, 'quaternion') and len(imu.quaternion) > 0 else 0,
                'imu_quaternion_x': imu.quaternion[1] if hasattr(imu, 'quaternion') and len(imu.quaternion) > 1 else 0,
                'imu_quaternion_y': imu.quaternion[2] if hasattr(imu, 'quaternion') and len(imu.quaternion) > 2 else 0,
                'imu_quaternion_z': imu.quaternion[3] if hasattr(imu, 'quaternion') and len(imu.quaternion) > 3 else 0,
            })
        
        # 电机状态数据
        if hasattr(self.low_state, 'motor_state') and self.low_state.motor_state:
            for i, motor in enumerate(self.low_state.motor_state):
                if i < 29:  # G1有29个电机
                    data_record.update({
                        f'motor_{i}_q': motor.q,
                        f'motor_{i}_dq': motor.dq,
                        f'motor_{i}_tau': motor.tau,
                        f'motor_{i}_mode': motor.mode,
                        f'motor_{i}_temperature': motor.temperature if hasattr(motor, 'temperature') else 0,
                    })
        
        # 机器人模式信息
        data_record.update({
            'mode_machine': self.low_state.mode_machine,
            'mode_pr': self.low_state.mode_pr if hasattr(self.low_state, 'mode_pr') else 0,
        })
        
        # 添加到数据缓冲区
        self.data_buffer.append(data_record)

    def LowCmdWrite(self):
        """发送控制命令（保持机器人稳定）"""
        self.time_ += self.control_dt_
        
        # 保持机器人稳定，不移动
        if self.low_state:
            for i in range(29):  # G1有29个电机
                self.low_cmd.mode_pr = 0  # PR模式
                self.low_cmd.mode_machine = self.mode_machine_
                self.low_cmd.motor_cmd[i].mode = 1  # 启用
                self.low_cmd.motor_cmd[i].tau = 0.0  # 无扭矩
                self.low_cmd.motor_cmd[i].q = self.low_state.motor_state[i].q  # 保持当前位置
                self.low_cmd.motor_cmd[i].dq = 0.0  # 无速度
                self.low_cmd.motor_cmd[i].kp = 60.0  # 位置增益
                self.low_cmd.motor_cmd[i].kd = 1.0   # 速度增益

        # 计算CRC并发送
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)

    def SaveData(self):
        """保存收集的数据为HDF5格式"""
        if not self.data_buffer:
            print("没有数据可保存")
            return
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.data_dir}/g1_data_{timestamp}.h5"
        
        self.SaveToHDF5(filename)
        
        print(f"数据已保存到: {filename}")
        print(f"总共收集了 {len(self.data_buffer)} 条数据")

    def SaveToHDF5(self, filename):
        """保存为HDF5格式"""
        if not self.data_buffer:
            return
            
        with h5py.File(filename, 'w') as hf:
            # 创建元数据组
            metadata = hf.create_group('metadata')
            metadata.attrs['robot_type'] = 'G1'
            metadata.attrs['total_records'] = len(self.data_buffer)
            metadata.attrs['collection_date'] = datetime.now().isoformat()
            metadata.attrs['sample_rate'] = 500  # 2ms间隔 = 500Hz
            
            # 创建数据组
            data_group = hf.create_group('data')
            
            # 获取所有字段名
            fieldnames = list(self.data_buffer[0].keys())
            
            # 为每个字段创建数据集
            for field in fieldnames:
                # 提取该字段的所有值
                values = [record[field] for record in self.data_buffer]
                
                # 转换为numpy数组
                values_array = np.array(values)
                
                # 创建数据集
                dataset = data_group.create_dataset(field, data=values_array)
                
                # 添加属性
                if 'timestamp' in field:
                    dataset.attrs['unit'] = 'seconds'
                    dataset.attrs['description'] = 'Unix timestamp'
                elif 'elapsed_time' in field:
                    dataset.attrs['unit'] = 'seconds'
                    dataset.attrs['description'] = 'Time elapsed since start'
                elif 'imu_rpy' in field:
                    dataset.attrs['unit'] = 'radians'
                    dataset.attrs['description'] = 'IMU roll, pitch, yaw'
                elif 'imu_gyro' in field:
                    dataset.attrs['unit'] = 'rad/s'
                    dataset.attrs['description'] = 'IMU angular velocity'
                elif 'imu_accel' in field:
                    dataset.attrs['unit'] = 'm/s²'
                    dataset.attrs['description'] = 'IMU acceleration'
                elif 'motor_' in field and '_q' in field:
                    dataset.attrs['unit'] = 'radians'
                    dataset.attrs['description'] = 'Motor position'
                elif 'motor_' in field and '_dq' in field:
                    dataset.attrs['unit'] = 'rad/s'
                    dataset.attrs['description'] = 'Motor velocity'
                elif 'motor_' in field and '_tau' in field:
                    dataset.attrs['unit'] = 'Nm'
                    dataset.attrs['description'] = 'Motor torque'

    def Stop(self):
        """停止数据收集"""
        if hasattr(self, 'lowCmdWriteThreadPtr'):
            self.lowCmdWriteThreadPtr.Stop()
        print("数据收集已停止")

    def GetDataSummary(self):
        """获取数据摘要"""
        if not self.data_buffer:
            return "没有数据"
            
        summary = {
            'total_records': len(self.data_buffer),
            'duration': self.data_buffer[-1]['elapsed_time'] if self.data_buffer else 0,
            'sample_rate': len(self.data_buffer) / (self.data_buffer[-1]['elapsed_time'] if self.data_buffer else 1),
            'data_fields': list(self.data_buffer[0].keys()) if self.data_buffer else []
        }
        
        return summary

def main():
    print("=== G1机器人数据收集器 ===")
    print("警告: 请确保机器人周围没有障碍物!")
    
    # 获取收集参数
    try:
        duration = float(input("请输入数据收集持续时间(秒，默认10秒): ") or "10")
    except ValueError:
        duration = 10.0
    
    print(f"将收集 {duration} 秒的数据，保存为HDF5格式")
    input("按Enter键开始...")

    # 初始化通道
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    # 创建数据收集器
    collector = G1DataCollector()
    
    try:
        # 初始化
        collector.Init()
        
        # 开始收集
        collector.Start(duration)
        
        # 等待收集完成
        while collector.time_ < duration:
            time.sleep(0.1)
        
        # 停止收集
        collector.Stop()
        
        # 显示数据摘要
        summary = collector.GetDataSummary()
        print("\n=== 数据收集摘要 ===")
        for key, value in summary.items():
            print(f"{key}: {value}")
        
        # 保存数据
        collector.SaveData()
        
    except KeyboardInterrupt:
        print("\n用户中断，正在停止...")
        collector.Stop()
        collector.SaveData()
    except Exception as e:
        print(f"发生错误: {e}")
        collector.Stop()
    finally:
        print("程序结束")

if __name__ == '__main__':
    main() 