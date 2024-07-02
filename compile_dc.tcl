#/**************************************************/
#/* Compile Script for Synopsys                    */
#/*                                                */
#/* dc_shell-t -f compile_dc.tcl                   */
#/*                                                */
#/* OSU FreePDK 45nm                               */
#/**************************************************/


#/* All verilog files, separated by spaces         */
set my_verilog_files [list LUMOS.v]

#/* Top-level Module                               */
set my_toplevel LUMOS

#/* The name of the clock pin. If no clock-pin     */
#/* exists, pick anything                          */
set my_clock_pin clk

#/* Target frequency in MHz for optimization       */
set my_clk_freq_MHz 500

#/* Delay of input signals (Clock-to-Q, Package etc.)  */
set my_input_delay_ns 0.1

#/* Reserved time for output signals (Holdtime etc.)   */
set my_output_delay_ns 0.1


#/**************************************************/
#/* Setup Library Files                            */
#/**************************************************/

set link_library "/FreePDK45/osu_soc/lib/files/gscl45nm.db"
set target_library "/FreePDK45/osu_soc/lib/files/gscl45nm.db"

#/**************************************************/
#/* Setup Design Library                           */
#/**************************************************/

define_design_lib WORK -path ./WORK

#/**************************************************/
#/* General Optimization Options                   */
#/**************************************************/
set verilogout_show_unconnected_pins "true"

#/**************************************************/
#/* Analyze and Elaboration                        */
#/**************************************************/

analyze -f verilog $my_verilog_files
elaborate $my_toplevel
current_design $my_toplevel

link
uniquify

#/**************************************************/
#/* Clock Definition & Constraints                 */
#/**************************************************/

set my_period [expr 1000 / $my_clk_freq_MHz]

set find_clock [ find port [list $my_clock_pin] ]
if {  $find_clock != [list] } {
   set clk_name $my_clock_pin
   create_clock -period $my_period $clk_name
} else {
   set clk_name vclk
   create_clock -period $my_period -name $clk_name
}

#/**************************************************/
#/* Input & Outputs Drive and Delay Options        */
#/**************************************************/

set_driving_cell  -lib_cell INVX1  [all_inputs]
set_input_delay $my_input_delay_ns -clock $clk_name [remove_from_collection [all_inputs] $my_clock_pin]
set_output_delay $my_output_delay_ns -clock $clk_name [all_outputs]


#/**************************************************/
#/* Compile Design                                 */
#/**************************************************/

compile_ultra

check_design

report_constraint -all_violators

#/**************************************************/
#/* Generate Netlist Verilog File                  */
#/**************************************************/

write -f verilog -hierarchy -output "LUMOS.vh"
write_sdc "LUMOS.sdc"
write -f db -hier -output "LUMOS.db"

#/**************************************************/
#/* Reports                                        */
#/**************************************************/

redirect reports/timing.rep { report_timing  -significant_digits 3}
redirect reports/cell.rep { report_cell }
redirect reports/area.rep { report_area }
redirect reports/power.rep { report_power }

quit
