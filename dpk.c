#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/kernel.h>

#include "imu20948.h"

#define DRIVER_NAME "imu20948"
#define DRIVER_CLASS "imu20948Class"

static struct i2c_adapter * imu_i2c_adapter = NULL;
static struct i2c_client * imu20948_i2c_client = NULL;

static struct gyroData gyro; // struct obj
IMU_ST_SENSOR_DATA gstGyroOffset = {0, 0, 0};
static struct accelData accel;



#define I2C_BUS_AVAILABLE       1               /* The I2C Bus available on the raspberry */
#define SLAVE_DEVICE_NAME       "IMU20948"        /* Device and Driver Name */
#define IMU20948_SLAVE_ADDRESS    0x68            /* BMP280 I2C address */

static const struct i2c_device_id imu_id[]={
{ SLAVE_DEVICE_NAME, 0},
{ }
};
static struct i2c_driver imu_driver = {
 .driver = {
   .name = SLAVE_DEVICE_NAME,
   .owner = THIS_MODULE
  }
};

static struct i2c_board_info imu20948_i2c_board_info = {
 I2C_BOARD_INFO(SLAVE_DEVICE_NAME, IMU20948_SLAVE_ADDRESS)
};


static dev_t myDeviceNr;
static struct class *myClass;
static struct cdev myDevice;

static void read_accel(void)
{

     uint8_t u8Buf[2];
    int16_t s16x,s16y,s16z;

    int32_t s32OutBuf[3] = {0};
    static ICM20948_ST_AVG_DATA sstAvgBuf[3];

    u8Buf[0] = i2c_smbus_read_byte_data(imu20948_i2c_client, REG_ADD_ACCEL_XOUT_L);
    u8Buf[1] = i2c_smbus_read_byte_data(imu20948_i2c_client, REG_ADD_ACCEL_XOUT_H);
     s16x = (u8Buf[1] << 8) | u8Buf[0];
  accel.x=  s16x;

    u8Buf[0] = i2c_smbus_read_byte_data(imu20948_i2c_client, REG_ADD_ACCEL_YOUT_L);
    u8Buf[1] = i2c_smbus_read_byte_data(imu20948_i2c_client, REG_ADD_ACCEL_YOUT_H);
    s16y = (u8Buf[1] << 8) | u8Buf[0];
  accel.y=  s16y;

    u8Buf[0] = i2c_smbus_read_byte_data(imu20948_i2c_client, REG_ADD_ACCEL_ZOUT_L);
    u8Buf[1] = i2c_smbus_read_byte_data(imu20948_i2c_client, REG_ADD_ACCEL_ZOUT_H);
    s16z = (u8Buf[1] << 8) | u8Buf[0];
  accel.z=  s16z;

      icm20948CalAvgValue(&sstAvgBuf[0].u8Index, sstAvgBuf[0].s16AvgBuffer, s16x, s32OutBuf + 0);
      icm20948CalAvgValue(&sstAvgBuf[1].u8Index, sstAvgBuf[1].s16AvgBuffer, s16y, s32OutBuf + 1);
      icm20948CalAvgValue(&sstAvgBuf[2].u8Index, sstAvgBuf[2].s16AvgBuffer, s16z, s32OutBuf + 2);
   
    accel.x = s32OutBuf[0];
    accel.y = s32OutBuf[1];
    accel.z = s32OutBuf[2];

}
void read_gyro(void)
  {
    uint8_t u8Buf[6];
    int16_t s16Buf[3] = {0};
    uint8_t i;
    int32_t s32OutBuf[3] = {0};
    static ICM20948_ST_AVG_DATA sstAvgBuf[3];
    static int16_t ss16c = 0;
    ss16c++;

    u8Buf[0] = i2c_smbus_read_byte_data(imu20948_i2c_client, REG_ADD_GYRO_XOUT_L);
    u8Buf[1] = i2c_smbus_read_byte_data(imu20948_i2c_client, REG_ADD_GYRO_XOUT_H);
    s16Buf[0] = (u8Buf[1] << 8) | u8Buf[0];

    u8Buf[0] = i2c_smbus_read_byte_data(imu20948_i2c_client, REG_ADD_GYRO_YOUT_L);
    u8Buf[1] = i2c_smbus_read_byte_data(imu20948_i2c_client, REG_ADD_GYRO_YOUT_H);
    s16Buf[1] = (u8Buf[1] << 8) | u8Buf[0];

    u8Buf[0] = i2c_smbus_read_byte_data(imu20948_i2c_client, REG_ADD_GYRO_ZOUT_L);
    u8Buf[1] = i2c_smbus_read_byte_data(imu20948_i2c_client, REG_ADD_GYRO_ZOUT_H);
    s16Buf[2] = (u8Buf[1] << 8) | u8Buf[0];

    for (i = 0; i < 3; i++)
    {
      icm20948CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
    }
    gyro.x = s32OutBuf[0] - gstGyroOffset.s16X;
    gyro.y = s32OutBuf[1] - gstGyroOffset.s16Y;
    gyro.z = s32OutBuf[2] - gstGyroOffset.s16Z;

    return;
  }
void icm20948GyroOffset(void)
  {
    uint8_t i;
    int16_t s16Gx = 0, s16Gy = 0, s16Gz = 0;
    int32_t s32TempGx = 0, s32TempGy = 0, s32TempGz = 0;
    for (i = 0; i < 32; i++)
    {
      icm20948GyroRead(&s16Gx, &s16Gy, &s16Gz);
      s32TempGx += s16Gx;
      s32TempGy += s16Gy;
      s32TempGz += s16Gz;
      delay(10);
    }
    gstGyroOffset.s16X = s32TempGx >> 5;
    gstGyroOffset.s16Y = s32TempGy >> 5;
    gstGyroOffset.s16Z = s32TempGz >> 5;
    return;
  }
    void icm20948CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal)
  {
    uint8_t i;

    *(pAvgBuffer + ((*pIndex)++)) = InVal;
    *pIndex &= 0x07;

    *pOutVal = 0;
    for (i = 0; i < 8; i++)
    {
      *pOutVal += *(pAvgBuffer + i);
    }
    *pOutVal >>= 3;
  }

void icm20948init(void)
  {
    /* user bank 0 register */
    i2c_smbus_write_byte_data(imu20948_i2c_client, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
    i2c_smbus_write_byte_data(imu20948_i2c_client, REG_ADD_PWR_MIGMT_1, REG_VAL_ALL_RGE_RESET);
    //delay(10);
    i2c_smbus_write_byte_data(imu20948_i2c_client, REG_ADD_PWR_MIGMT_1, REG_VAL_RUN_MODE);

    /* user bank 2 register */
    i2c_smbus_write_byte_data(imu20948_i2c_client, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2);
    i2c_smbus_write_byte_data(imu20948_i2c_client, REG_ADD_GYRO_SMPLRT_DIV, 0x07);
    i2c_smbus_write_byte_data(imu20948_i2c_client, REG_ADD_GYRO_CONFIG_1,
                     REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_1000DPS | REG_VAL_BIT_GYRO_DLPF);
    i2c_smbus_write_byte_data(imu20948_i2c_client, REG_ADD_ACCEL_SMPLRT_DIV_2, 0x07);
    i2c_smbus_write_byte_data(imu20948_i2c_client, REG_ADD_ACCEL_CONFIG,
                     REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g | REG_VAL_BIT_ACCEL_DLPF);

    /* user bank 0 register */
    i2c_smbus_write_byte_data(imu20948_i2c_client, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);

    icm20948GyroOffset();

    return;
  }

static ssize_t driver_read(struct file *File, char __user *user_buffer, size_t count, loff_t *offs)
{
 int to_copy, not_copied, delta;
 char out_string[73];
 int16_t s16x,s16y,s16z,s16gx,s16gy,s16gz;


 to_copy = min(sizeof(out_string), count);

 read_accel();
 s16x = accel.x;
 s16y = accel.y;
 s16z = accel.z;

 read_gyro();
 s16gx=gyro.x;
 s16gy=gyro.y;
 s16gz=gyro.z;

 snprintf(out_string, sizeof(out_string), "accel_x:%d,y:%d,z:%d\ngyro_x:%d,y:%d,z:%d\n",s16x,s16y,s16z,s16gx,s16gy,s16gz);
 not_copied = copy_to_user(user_buffer, out_string, to_copy);

 delta = to_copy - not_copied;

 return delta;
}

static int driver_open(struct inode *deviceFile, struct file *instance)
{
 printk("Driver Open\n");
 return 0;
}

static int driver_close(struct inode *deviceFile, struct file *instance)
{
        printk("Driver Close\n");
        return 0;
}

static struct file_operations fops  = {
 .owner = THIS_MODULE,
 .open = driver_open,
 .release = driver_close,
 //.unlocked_ioctl = ioctl_dev,
 .read = driver_read,
};

static int __init ModuleInit(void) {
  int ret = -1;
  u8 id;
  printk("MyDeviceDriver - Hello Kernel\n");

  /* Allocate Device Nr */
  if ( alloc_chrdev_region(&myDeviceNr, 0, 1, DRIVER_NAME) < 0) /////////////////////////////////////
  {
 printk("Device Nr. could not be allocated!\n");
  }
  printk("MyDeviceDriver - Device Nr %d was registered\n", myDeviceNr);

  /* Create Device Class */
  if ((myClass = class_create(THIS_MODULE, DRIVER_CLASS)) == NULL) {
    printk("Device Class can not be created!\n");
    goto ClassError;
  }

  /* Create Device file */
  if (device_create(myClass, NULL, myDeviceNr, NULL, DRIVER_NAME) == NULL)
  {
    printk("Can not create device file!\n");
    goto FileError;
  }

  /* Initialize Device file */
  cdev_init(&myDevice, &fops);

  /* register device to kernel */
  if (cdev_add(&myDevice, myDeviceNr, 1) == -1) {
    printk("Registering of device to kernel failed!\n");
    goto KernelError;
  }

  imu_i2c_adapter = i2c_get_adapter(I2C_BUS_AVAILABLE);

  if(imu_i2c_adapter != NULL) {
    imu20948_i2c_client = i2c_new_client_device(imu_i2c_adapter, &imu20948_i2c_board_info);
    if(imu20948_i2c_client != NULL) {
      if(i2c_add_driver(&imu_driver) != -1) {
        ret = 0;
      }
      else
        printk("Can't add driver...\n");
    }
    i2c_put_adapter(imu_i2c_adapter);
  }
 printk("IMU20948 Driver added!\n");

  /* Read Chip ID */
  id = i2c_smbus_read_byte_data(imu20948_i2c_client, 0x00);
  printk("ID: 0x%x\n", id);

  icm20948init();

  return ret;

KernelError:
  device_destroy(myClass, myDeviceNr);
FileError:
  class_destroy(myClass);
ClassError:
  unregister_chrdev(myDeviceNr, DRIVER_NAME);
  return (-1);
}

/**
 * @brief function, which is called when removing module from kernel
 * free alocated resources
 */
static void __exit ModuleExit(void) {
  printk("MyDeviceDriver - Goodbye, Kernel!\n");
  i2c_unregister_device(imu20948_i2c_client);
  i2c_del_driver(&imu_driver);
  cdev_del(&myDevice);
    device_destroy(myClass, myDeviceNr);
    class_destroy(myClass);
    unregister_chrdev_region(myDeviceNr, 1);
}

module_init(ModuleInit);
module_exit(ModuleExit);

