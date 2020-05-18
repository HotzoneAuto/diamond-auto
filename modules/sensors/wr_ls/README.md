#wr_ls
# HOW TO RUN
# 1: catkin_make
# 2: catkin_make install
# 3: roslaunch wr_ls wr_ls1207de.launch
#
# HOW TO VIEW FROM RVIZ
# 1: Run the package as introduced .
# 2: Start another terminal.
# 3: rosrun rviz rviz
# 4: In the popup window, click [Add] on the left-bottum
# 5: In the popup window, select  [By topic] tab.
# 6: Select [LaserScan], then [OK]
# 7: Back to the main window, in the left operation panel:
     [Displays]->[Global Options]->[Fixed Frame]->Change the value of this item to [laser]
# 8: Then, you should be able to see the result in display window.
#
# HOW TO DISABLE DEBUG MODE
# 1: In cfg/WrLs.cfg
# 2: Set the default value of debug_mode to False

# HOW TO DISABLE CHECK FRAME MODE
# 1: In launch/wr_ls1207de.launch
# 2: Set the checkframe = false in xml 
