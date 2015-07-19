bool stop_flag #阻挡标志，当此标志为true时，飞行器向前飞过（dx,dy）后，判断(sum_x == sum_x_ref && velocity == 0)==true时，飞行器下降，执行阻挡irobot功能
float32 dx
float32 dy
