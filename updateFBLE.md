| Command      | Parameter                     | Example        | Description                                                                 |
| :----------- | :---------------------------- | :------------- | :-------------------------------------------------------------------------- |
| kp=[value] | Kp (Proportional Gain)        | kp=3.1       | Sets the Kp value (float).                                                  |
| ki=[value] | Ki (Integral Gain)            | ki=0.05      | Sets the Ki value (float).                                                  |
| kd=[value] | Kd (Derivative Gain)          | kd=35        | Sets the Kd value (float).                                                  |
| ms=[value] | Max Straight Speed            | ms=250       | Sets max_straight_speed (integer, 0-255).                                 |
| cs=[value] | Cornering Speed               | cs=200       | Sets cornering_speed (integer, 0-255).                                    |
| maxerr=[value] | Max Error for High Speed    | maxerr=2.0   | Sets max_error_for_high_speed (float). Error must be below this for max speed. |
| minerr=[value] | Min Error for Low Speed     | minerr=4.0   | Sets min_error_for_low_speed (float). Error must be above this for cornering speed. |
| sharpred=[value] | Sharp Turn Speed Reduction | sharpred=40  | Sets sharp_turn_speed_reduction (integer). Amount speed is reduced on sharp turns. |
| corrscale=[value] | Correction Scale           | corrscale=55 | Sets correction_scale (integer). Scales PID output to motor difference. |
| bypdur=[value] | Bypass Turn Duration        | bypdur=600   | Sets bypass_turn_duration (integer, milliseconds).                        |
| threshblk=[value] | Threshold Black            | threshblk=350| Sets threadshold_black (integer). IR value below this is considered black. |
| getpid     | (N/A)                         | getpid       | Requests the current Kp, Ki, Kd values via notification.                    |
| getall     | (N/A)                         | getall       | Requests all current tunable values via notification.                       |