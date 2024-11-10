## Setup Instructions

**Follow the steps below to set up your robots in RoboDK and configure the GH file:**

1)Open RoboDK

2)Open Robot Library
Choose your two robots of choice.
Ensure both robots are active in RoboDK.
Rename the robots if desired.

3)Open the GH File

4)Using the Human Plugin you can retrieve object from specific layers without having to link each time and here you can set the points that will be the candidates of your robot base
![tablepoints](https://github.com/user-attachments/assets/f64baf8c-663e-48b1-adca-81302f91b134)

5)Choose Mode
Select between Manual or External mode depending on how you want to set up your inputs.

![robotinputs](https://github.com/user-attachments/assets/33c244af-c827-47a1-97fa-afdf357a9a59)
Manual Mode Setup

If you chose Manual:
Enter the IDs of your two robots.
Specify the locations of their bases.
Define the orientation of their reference frames.

6)After all of your inputs are set in you can use the RobotSetup.py script to connect the active the robots from robo dk to the relative positions and orientation of the rhino environment.
![RobotBase](https://github.com/user-attachments/assets/4be64b7b-d47a-448d-9230-3b620e69c0bc)

7)Same with the Assembly sequence script named Trussemble.py script, using Human plugin you can get the objects you need from the respective rhino layers to get ordered assembly steps.
![assemblysequence](https://github.com/user-attachments/assets/a08c0425-078e-4827-8969-a9618b2ebafb)

8)Afterwards you can visualise each assembly step of your structure.
![steps](https://github.com/user-attachments/assets/a730c1d2-0879-410f-b22e-aed2b5ba0c25)

9)Then you can enter the reach of your robots and combined with the results of the assembly sequence algorithm you can distribute the elements to corresponding robot using the TaskDistribution.py
![task distribution](https://github.com/user-attachments/assets/b64115b6-3cfd-444e-a0cf-5a06e25eb4c0)

10)Afterwards the planes will start to get assigned to the elements but you can modify them, the pufferfish plugin helps a lot with the plane setup and orientation.

![planes](https://github.com/user-attachments/assets/5cccedff-e9ad-4ad6-bbf6-d54ce5a05ccf)

11)Here you need to press play so the connection between robo dk and rhino is established and you can retrieve necessary information like the home planes and the joint positions which are necessary for further calculation. Make sure you press stop after a while so the home planes don't get overwritten.And make sure the robots are in home position in the robodk environment.

![realtime](https://github.com/user-attachments/assets/de9767a2-5dc4-43f0-9308-8071bd5275fc)

12)Here all the resulted planes get inputed and through the RobotCommand.py script they get ordered and distributed to the corrresponding robot giving the necessary plane list to work with.

![RobotCommand](https://github.com/user-attachments/assets/9f211110-6701-4233-9a1a-7eb8a888ca18)

**If there are planes you wish to remove before the creation of the robot program you can do so to prevent issues**

![removal](https://github.com/user-attachments/assets/049bebf0-cc5a-499f-89bc-683ff15ad53d)

13)Here is the final step where the robot programs are created and exported to RoboDk

![cycle](https://github.com/user-attachments/assets/1cc8ed3a-bb03-4c08-a290-15423ec04d71)













