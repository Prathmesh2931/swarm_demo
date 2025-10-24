// FINAL SUBMITTED 38 MARKS CODE
/*This cpp file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/pid_error			    /throttle_pid
		            			/pitch_pid
		            			/roll_pid
					
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.*/

// importing the required libraries
#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <algorithm>

#include "geometry_msgs/msg/pose_array.hpp"
#include "swift_msgs/msg/swift_msgs.hpp"
#include "controller_msg/msg/pid_tune.hpp"
#include "error_msg/msg/error.hpp"  
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

struct CMD
{
    int rc_roll;
    int rc_pitch;
    int rc_yaw;
    int rc_throttle;
    int rc_aux4;
};

struct ERROR
{
    float roll_error;
    float pitch_error;
    float throttle_error;
    float yaw_error;

};


class Swift_Pico : public rclcpp::Node
{
    public:
        Swift_Pico() : Node("pico_controller") //initializing ros node with name pico_controller
        {
            /* This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		     [x,y,z]*/
            drone_position[0] = 0.0;
            drone_position[1] = 0.0;
            drone_position[2] = 0.0;

            /* [x_setpoint, y_setpoint, z_setpoint]
            whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly*/
            setpoint[0] = -7;
            setpoint[1] = 0;
            setpoint[2] = 20;

            //Declaring a cmd of message type swift_msgs and initializing values
            cmd.rc_roll = 1500;
            cmd.rc_pitch = 1500;
            cmd.rc_yaw = 1500;
            cmd.rc_throttle = 1500;

            /*initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		    after tuning and computing corresponding PID parameters, change the parameters*/

            //proportional
            Kp[0] = 0;
            Kp[1] = 0;
            // Kp[2] = 0;
            Kp[2] = 60.5;
            // Kp[2] = 14.1;

            //integral
            Ki[0] = 0;
            Ki[1] = 0;
            // Ki[2] = 0;
            Ki[2] = 0.108;
            // Ki[2] = 0.104;

            //derivative
            Kd[0] = 0;
            Kd[1] = 0;
            // Kd[2] = 0;
            Kd[2] = 1600;
            // Kd[2] = 282;
            /*-----------------------Add other required variables for pid here ----------------------------------------------*/

            prev_error[0] = 0.0f; prev_error[1] = 0.0f; prev_error[2] = 0.0f; // [roll, pitch, throttle]
            error_sum[0] = 0.0f; error_sum[1] = 0.0f; error_sum[2] = 0.0f;    //  [roll, pitch, throttle]

            max_values[0] = 2000; max_values[1] = 2000; max_values[2] = 5000;
            min_values[0] = 1000; min_values[1] = 1000; min_values[2] = 1000;


            /* Hint : Add variables for storing previous errors in each axis, like prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]. 
            Add variables for limiting the values like max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
            										   min_values = [1000,1000,1000] corresponding to [roll, pitch, throttle]
            Add variables for publishing error.
            You can change the upper limit and lower limit accordingly. 
            ---------------------------------------------------------------------------------------------------------*/

            /*This is the sample time in which you need to run pid. Choose any time which you seem fit.*/
        
            sample_time = 40ms; //in milli-seconds

            //Publishing /drone_command, /throttle_error, /pitch_error, /roll_error
            command_pub = this->create_publisher<swift_msgs::msg::SwiftMsgs>("/drone_command", 10);
            pid_error_pub = this->create_publisher<error_msg::msg::Error>("/pid_error", 10);


            //Add other ROS 2 Publishers here

            //Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, roll_pid
            whycon_sub = this->create_subscription<geometry_msgs::msg::PoseArray>("/whycon/poses", 1, std::bind(&Swift_Pico::whycon_callback, this, _1));
            throttle_pid_sub = this->create_subscription<controller_msg::msg::PIDTune>("/throttle_pid", 1, std::bind(&Swift_Pico::altitude_set_pid, this, _1));

            //------------------------Add other ROS 2 Subscribers here-----------------------------------------------------
            roll_pid_sub = this->create_subscription<controller_msg::msg::PIDTune>("/roll_pid", 1, std::bind(&Swift_Pico::roll_set_pid, this, _1));
            pitch_pid_sub = this->create_subscription<controller_msg::msg::PIDTune>("/pitch_pid", 1, std::bind(&Swift_Pico::pitch_set_pid, this, _1));

            //Arming the drone
            arm();

            //Creating a timer to run the pid function periodically, refer ROS 2 tutorials on how to create a publisher subscriber(C++)
            pid_timer_ = this->create_wall_timer(sample_time, std::bind(&Swift_Pico::pid, this));

        }

    private:

        //declare all the variables, arrays, strcuts etc. here
        float drone_position[3];
        int setpoint[3];
        swift_msgs::msg::SwiftMsgs cmd;
        std::chrono::milliseconds sample_time;
        float Kp[3];
        float Ki[3];
        float Kd[3];
        
        //variables for PID control
        float prev_error[3]; 
        float error_sum[3];
        float max_values[3];    // max values for [roll, pitch, throttle]
        float min_values[3];    // min values for [roll, pitch, throttle]

        float out_roll;
        float out_pitch;
        float out_throttle;

        // NEW VARIABLES FOR D-TERM FILTER
        float d_filter_alpha = 0.6f; // Filter coefficient (0=max filter, 1=no filter)
        float filtered_derivative[3] = {0.0f, 0.0f, 0.0f};

        //declare the publishers and subscribers here
        rclcpp::Publisher<swift_msgs::msg::SwiftMsgs>::SharedPtr command_pub;
        rclcpp::Publisher<error_msg::msg::Error>::SharedPtr pid_error_pub;

        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr whycon_sub;
        rclcpp::Subscription<controller_msg::msg::PIDTune>::SharedPtr throttle_pid_sub;
        rclcpp::Subscription<controller_msg::msg::PIDTune>::SharedPtr pitch_pid_sub;
        rclcpp::Subscription<controller_msg::msg::PIDTune>::SharedPtr roll_pid_sub;

        // ROS2 Timer
        rclcpp::TimerBase::SharedPtr pid_timer_;

        //define functions and callbacks here

        void disarm()
        {
            auto cmd = swift_msgs::msg::SwiftMsgs();
            cmd.rc_roll = 1000;
            cmd.rc_pitch = 1000;
            cmd.rc_yaw = 1000;
            cmd.rc_throttle = 1000;
            cmd.rc_aux4 = 1000;
            command_pub->publish(cmd);
        }

        void arm()
        {   
            auto cmd = swift_msgs::msg::SwiftMsgs();
            disarm();
            cmd.rc_roll = 1500;
            cmd.rc_pitch = 1500;
            cmd.rc_yaw = 1500;
            cmd.rc_throttle = 1500;
            cmd.rc_aux4 = 2000;
            command_pub->publish(cmd);
        }

        void whycon_callback(const geometry_msgs::msg::PoseArray & msg)
        {
            // the first pose in the array is the drone's pose.
            if (msg.poses.empty()) return; // Safety check for empty message

            drone_position[0] = msg.poses[0].position.x;
            
            // my code
            //--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
            drone_position[1] = msg.poses[0].position.y;
            drone_position[2] = msg.poses[0].position.z;
	
		    //------------------------------------------------------------------------------------------------------------------------
        }

        //----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
        //[roll, pitch, throttle]
        
        //Callback function for /roll_pid
        void roll_set_pid(const controller_msg::msg::PIDTune & roll)
        {
            Kp[0] = roll.kp * 0.03;  // This is just for an example. You can change the ratio/fraction value accordingly
		    Ki[0] = roll.ki * 0.008;
		    Kd[0] = roll.kd * 0.6;
        }

        //Callback function for /pitch_pid
        void pitch_set_pid(const controller_msg::msg::PIDTune & pitch)
        {
            Kp[1] = pitch.kp * 0.03;  // This is just for an example. You can change the ratio/fraction value accordingly
		    Ki[1] = pitch.ki * 0.008;
		    Kd[1] = pitch.kd * 0.6;
        }

        //Callback function for /throttle_pid
	    //This function gets executed each time when /drone_pid_tuner publishes /throttle_pid
	
        void altitude_set_pid(const controller_msg::msg::PIDTune & alt)
        {
            // Kp[2] = alt.kp * 0.03;  // This is just for an example. You can change the ratio/fraction value accordingly
		    // Ki[2] = alt.ki * 0.008;
		    // Kd[2] = alt.kd * 0.6;
            (void)alt;
        }

        //----------------------------------------------------------------------------------------------------------------------


        void pid()
        {
                
            auto error_pub = error_msg::msg::Error();
            /*-----------------------------Write the PID algorithm here--------------------------------------------------------------

            # Steps:
            # 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
            #	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
            #	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
            #	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
            #	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
            #	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
            #																														self.cmd.rcPitch = self.max_values[1]
            #	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
            #	8. Add error_sum
            
            */

            // Step 1: Compute error. Note the axis mapping: Pitch corrects X, Roll corrects Y.
            // Index mapping: [0] = Roll, [1] = Pitch, [2] = Throttle
            float error[3];
            error[0] =  setpoint[0] - drone_position[0];
            error[1] = drone_position[1] - setpoint[1];
            error[2] = setpoint[2] - drone_position[2];

            // Step 2 & 3: Compute P, I, D terms and total output
            // Proportional
            float p_out[3];
            p_out[0] = Kp[0] * error[0];
            p_out[1] = Kp[1] * error[1];
            p_out[2] = Kp[2] * error[2];

            // Integral with anti-windup
            error_sum[0] += error[0];
            error_sum[1] += error[1];
            error_sum[2] += error[2];
            error_sum[0] = ((error_sum[0] < -400.0f) ? -400.0f : (error_sum[0] > 400.0f) ? 400.0f : error_sum[0]);
            error_sum[1] = ((error_sum[1] < -400.0f) ? -400.0f : (error_sum[1] > 400.0f) ? 400.0f : error_sum[1]);
            error_sum[2] = ((error_sum[2] < -400.0f) ? -400.0f : (error_sum[2] > 400.0f) ? 400.0f : error_sum[2]);

            float i_out[3];
            i_out[0] = Ki[0] * error_sum[0];
            i_out[1] = Ki[1] * error_sum[1];
            i_out[2] = Ki[2] * error_sum[2];

            // Derivative
            // float d_out[3];
            // d_out[0] = Kd[0] * (error[0] - prev_error[0]);
            // d_out[1] = Kd[1] * (error[1] - prev_error[1]);
            // d_out[2] = Kd[2] * (error[2] - prev_error[2]);

            // Derivative with Low-Pass Filter
            float d_out[3];
            float raw_derivative[3];

            // Calculate raw (noisy) derivative for all axes
            raw_derivative[0] = error[0] - prev_error[0];
            raw_derivative[1] = error[1] - prev_error[1];
            raw_derivative[2] = error[2] - prev_error[2];

            // Apply the exponential moving average filter
            for (int i = 0; i < 3; ++i) {
                filtered_derivative[i] = (d_filter_alpha * filtered_derivative[i]) + ((1 - d_filter_alpha) * raw_derivative[i]);
                d_out[i] = Kd[i] * filtered_derivative[i];
            }

            // Total PID output
            float out_roll = p_out[0] + i_out[0] + d_out[0];
            float out_pitch = p_out[1] + i_out[1] + d_out[1];
            float out_throttle = p_out[2] + i_out[2] + d_out[2];

            // Step 4: Apply output to the neutral command value (1500)
            this->cmd.rc_roll = 1500 + static_cast<int>(out_roll);
            this->cmd.rc_pitch = 1500 + static_cast<int>(out_pitch);
            this->cmd.rc_throttle = 1500 - static_cast<int>(out_throttle);

            // Step 6: Limit the final command values
            this->cmd.rc_roll = (this->cmd.rc_roll < min_values[0]) ? min_values[0] : ((this->cmd.rc_roll > max_values[0]) ? max_values[0] : this->cmd.rc_roll);
            this->cmd.rc_pitch = (this->cmd.rc_pitch < min_values[1]) ? min_values[1] : ((this->cmd.rc_pitch > max_values[1]) ? max_values[1] : this->cmd.rc_pitch);
            this->cmd.rc_throttle = (this->cmd.rc_throttle < min_values[2]) ? min_values[2] : ((this->cmd.rc_throttle > max_values[2]) ? max_values[2] : this->cmd.rc_throttle);

            // Update the previous error
            prev_error[0] = error[0];
            prev_error[1] = error[1];
            prev_error[2] = error[2];

            // #------------------------------------------------------------------------------------------------------------------------
            command_pub->publish(this->cmd);
            error_pub.roll_error = error[0];
            error_pub.pitch_error = error[1];
            error_pub.throttle_error = error[2];
            //calculate throttle error, pitch error and roll error, then publish it accordingly
            pid_error_pub->publish(error_pub);

        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Swift_Pico>());
    rclcpp::shutdown();
    return 0;
}