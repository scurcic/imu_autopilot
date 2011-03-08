void adns9500_1_write_reg(unsigned char reg,unsigned char data);
void adns9500_2_write_reg(unsigned char reg,unsigned char data);

void adns9500_on_spi_write_reg(void);

void adns9500_1_srom_download(void);
void adns9500_2_srom_download(void);

void adns9500_1_write_srom(void);
void adns9500_2_write_srom(void);

void my_delay(unsigned long delay );
void delay_us(unsigned long us);

void adns9500_1_power_up_sequence(void);
void adns9500_2_power_up_sequence(void);

void adns9500_1_read_reg(unsigned char reg);
void adns9500_2_read_reg(unsigned char reg);

int adns9500_1_get_value(int axis);
int adns9500_2_get_value(int axis);

void adns9500_init(void);

void adns9500_1_select(void);
void adns9500_1_unselect(void);

void adns9500_2_select(void);
void adns9500_2_unselect(void);

void us_delay(unsigned long us);


void adns9500_on_spi_motion_burst(void);

unsigned char set_bits(unsigned char data,unsigned char mask);
unsigned char clear_bits(unsigned char data,unsigned char mask);
void adns9500_on_spi_write(void);
void adns9500_1_on_spi_read_reg(void);
void adns9500_2_on_spi_read_reg(void);

void adns9500_on_spi_write_srom(void);

void adns9500_1_read_sensor(void);
void adns9500_2_read_sensor(void);

void check_connection(void);

//void adns9500_1_check_motion(void);
//void adns9500_2_check_motion(void);

void adns9500_read_motion_burst(void);

//Registers
#define PRODUCT_ID	0x00
#define REVISION_ID	0x01
#define MOTION	0x02
#define DELTA_X_L	0x03
#define DELTA_X_H	0x04
#define DELTA_Y_L	0x05
#define DELTA_Y_H	0x06
#define SQUAL	0x07
#define PIXEL_SUM	0x08
#define MAXIMUM_PIXEL	0x09
#define MINIMUM_PIXEL	0x0A
#define SHUTTER_LOWER	0x0B
#define SHUTTER_UPPER	0x0C
#define FRAME_PERIOD_LOWER	0x0D
#define FRAME_PERIOD_UPPER	0x0E
#define CONFIGURATION_I	0x0F
#define CONFIGURATION_II	0x10
#define FRAME_CAPTURE	0x12
#define SROM_ENABLE	0x13
#define RUN_DOWNSHIFT	0x14
#define REST1_RATE	0x15
#define REST1_DOWNSHIFT	0x16
#define REST2_RATE	0x17
#define REST2_DOWNSHIFT	0x18
#define REST3_RATE	0x19
#define FRAME_PERIOD_MAX_BOUND_LOWER	0x1A
#define FRAME_PERIOD_MAX_BOUND_UPPER	0x1B
#define FRAME_PERIOD_MIN_BOUND_LOWER	0x1C
#define FRAME_PERIOD_MIN_BOUND_UPPER	0x1D
#define SHUTTER_MAX_BOUND_LOWER	0x1E
#define SHUTTER_MAX_BOUND_UPPER	0x1F
#define LASER_CTRL0	0x20
#define OBSERVATION	0x24
#define DATA_OUT_LOWER	0x25
#define DATA_OUT_UPPER	0x26
#define SROM_ID	0x2A
#define LIFT_DETECTION_THR	0x2E
#define CONFIGURATION_V	0x2F
#define CONFIGURATION_IV	0x39
#define POWER_UP_RESET	0x3A
#define SHUTDOWN	0x3B
#define INVERSE_PRODUCT_ID	0x3F
#define MOTION_BURST	0x50
#define SROM_LOAD_BURST	0x62
#define PIXEL_BURST	0x64

// other definitions
#define MASK	0x80
