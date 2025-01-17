#include "MTT/includes.h"

//uart0 - wheel1
//uart5 - wheel2

volatile int can_check = 0, rep_check = 0, temp = 0;
volatile float angle_offset = 0, scaling_f =35;

volatile float enc1_max=696.2, enc2_max=220.5;
volatile float temp_kp=0.04, temp_ki=0.0000015, temp_kd=0.0005;


void ig32_isr()
{
    intclear(timer1);

    pid_init(&orient, temp_kp, temp_ki, temp_kd, 0);

    if (angle_correct)
        angle_correction();
    rpm1 = w*scaling_f;
    rpm2 = w*scaling_f;


    if(rpm_control)
    {
        Rpm_Motor_control( rpm1, &motor_h0_lmd, uart0_count*(550*60/enc1_max));
        Rpm_Motor_control( rpm2, &motor_h3_lmd, uart1_count*(550*60/enc2_max));

        uart0_count = uart1_count = 0;
    }
    else
    {
        pwm_duty_rep(H0,0,1,0);
        pwm_duty_rep(H3,0,1,0);

        rpm1 = 0;
        rpm2 = 0;

//        uart0_count = uart1_count = 0;
    }
    rep_check++;
}

int main(void)
{
    System80Mhz();

    int_disable();

    lmd_H0_init();
    lmd_H3_init();

    imu_init(uart4);

    encoder_init(uart0_enc|uart1_enc);

    pid_init(&orient, temp_kp, temp_ki, temp_kd, 0);

    timer_init(timer1, 550, ig32_isr);

//    can_init(can_isr);

    int_enable();

    uart0_dir = 1;
    uart1_dir = -1;

    rpm_control = 1;

    while(1)
    {

    }
}
