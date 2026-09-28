"# Silvershield-mmWave" 

The purpose of this project is to develop a privacy-preserving, non-intrusive monitoring system using mmWave radar and Edge AI to enable real-time fall detection and activity recognition for senior care.

Section 1: Software Side (IWR6843ISK)

This code is based off Texas Instrument's (TI) 3D people tracking demo. I have added the following to meet the requirements of the project:

State-machine based fall detection logic

The radar receives these echoes and calculates three things for every "dot" it sees:

•Range: How far away is it?

•	Doppler: How fast is it moving toward or away from the radar?

•	Angle: Where is it horizontally (Azimuth) and vertically (Elevation)?

The radar does not just see a "person"; it sees a cloud of 50 to 100 points. The Tracking Algorithm groups these points together and assigns them a Target ID (like "Person 5").

For every frame (about every 50 to 100ms), the radar looks at the "shape" of that point cloud cluster. It extracts specific features to determine if a fall is happening:

•	Centroid Height (currentZ): The average height of the person. 

•	Vertical Velocity (velocityZ): How fast is the height changing? A controlled sit-down is slow while a fall is a rapid acceleration toward the floor.

•	Aspect Ratio: A standing person is tall and thin (high aspect ratio). A fallen person is wide and flat (low aspect ratio).

Using if/else if logic, the radar moves the Target ID through different states based on thresholds:

•	Normal State: The centroid height is above a certain threshold such as currentZ > 0.6m.

•	Falling State: The vertical velocity exceeds a "danger" threshold velocityZ < -1.5 m/s, and the height is dropping rapidly.

•	Fallen State: The height has stayed low such as currentZ < 0.4m for a set amount of time (usually 1 - 2 seconds) to avoid "false positives" from someone just picking up a pen.

Once the State Machine decides a state change has occurred, it triggers the code I have added:

•	Detection: The algorithm flags FALL_DETECTED for Target ID: 5.

• Formatting: sprintf takes that raw integer 5 and wraps it in a human-readable string: "FALL:5\r\n".

• Transmission: UART_write pushes those ASCII bytes out of the hardware pins, down the USB cable, and into your Python script.
 
Entry/exit detection state-machine system

I was requested to add a system to detect whether a person had entered or exited the room. This system is also a state-machine based system. It would print whether the target has entered or exited the room on the Python Emulation Script.

Software Side: Simulate Nordic Board (for reference only)

Python Emulation Script

To overcome hardware availability constraints, I developed a Python-based emulation script to simulate sensor data streams.
This enables rigorous firmware testing and edge-case validation without requiring physical hardware and significantly accelerated the debugging cycle and allowed for parallel development of software and hardware modules.

This script includes timestamps for each target and a 'Current people in the room' indicator.

Software Side: nRF5340 (All the code here is on the nRF5340, and includes code for the acoustic sensor)

Coding onto the nRF5340 (Implementing Bluetooth for wireless communication with GSM Modem + Acoustic Sensor (EV_T5837-FX2 with Infineon IM66D132HV01 FLEX) to detect thuds and distress calls). (hello_world_BACKUP)

For this portion of the project, I needed to first connect the IWR6843ISK mmWave radar to the nRF5340 for them to be able to serially communicate with each other. P0.26 goes to RX of IWR6843ISK, MSS_LOGGER connects to P0.27 and we must connect a common ground. The code enables Bluetooth communication to take the data and information from the IWR6843ISK (Entry, Exit, Falls, Recover) and sends them to the GSM Modem in Hex codes as they are more lightweight. An acoustic sensor is also connected to the nRF5340, I have coded it to trigger a heavy impact event when the microphone detects a noise louder than 3000 decibels, and used Edge AI to allow the nRF5340 to detect when someone says 'Help'.

All outputs can be seen when the device is connected to COM7 via the Terminal. Each event has a corresponding hex code that will be sent to the GSM Modem when the conditions are met:

RADAR_EVENT_ENTRY    = 0x01,

RADAR_EVENT_EXIT     = 0x02,

RADAR_EVENT_FALL     = 0x03,

RADAR_EVENT_RECOVERY = 0x04,

AUDIO_EVENT_DROP     = 0x05, 

AUDIO_EVENT_HELP     = 0x06 

There is also a 'DEBUG: Live Peak Volume: ' line that would continuously print. This line allows us to see what the EV_T5837-FX2 hears, if it passes the 3000 decibel condition, it would trigger event 0x05. 

Software Side: Edge Impulse

When training the AI model in Edge Impulse, I had to give it 3 distinct samples, noise, such as AC units running or tap water flowing, unknown such as people talking and help, which is people saying 'help'. We must ensure each sample has an approximately equal amount of audio to ensure there is no imbalance during training. I have kept each sample to 5 minutes, but longer duration and more variety would improve the AI's learning and functionality. The noise and unknown samples have been directly taken from Edge Impulse's 'Keyword Spotting' Sample, at the bottom of this README.md, you will see the citation as requested by Edge Impulse. When uploading samples into Edge Impulse, do ensure the audio files are in .WAV format, as .MP4 is only for videos. After you have uploaded all your data, you need to go to the left sidebar, and click 'create impulse'. From there, make sure to set Window size to 1000 ms, set Window increase to 500 ms, click Add a processing block and Choose Audio (MFCC). Click Add a learning block and choose Classification (Keras). After that, save the impulse. Now go back to the sidebar, click MFCC (It is right under Create Impulse), leave everything as the default since it is already optimised for Voice Recognition. Click 'Save parameters' and Edge Impulse will direct you to a new page where you can Generate Features. You will see a heatmap, it shows which data sounds similar to the AI, once the cloud servers parses finish the data, you will obtain the confusion matrix. 

Each part of the confusion matrix represents the following: The Rows (Left Side): The actual audio. The Columns (Top): What the AI predicted it was. The Green Diagonal: The correct answers (True Positives). After you see the AI's score, you can test the AI. Edge Impulse will take a certain percentage and train the AI, this value can be set by you before uploading the data to the model. I have set mine to the default 80% training 20% testing. The AI will be tested on the new 20% that it has never seen before. This will give you a new confusion matrix. If you are satisfied with the AI's results (>85% accuracy), you can generate the code libraries, which then we can upload into Visual Studio code folder so the AI can be flashed onto the nRF5340. 

Software Side: Acoustic Sensor 'Help' Detection on nRF5340

Along with the 'DEBUG: Live Peak Volume', you will also see something similar to these lines being printed continuously:

help: 0.01562

noise: 0.97266

unknown: 0.01172

This is the Edge AI's confidence score for each category. Event 0x06 will only trigger if the AI is 80% confident it heard a 'help' (0.8). This is done to ensure it does not trigger on words that sound similar to 'help' such as 'hello' or 'yelp'. One issue you may face is if you scream/shout help to close to the microphone, it would deem it as a heavy impact (0x05). To fix this, you could train the AI to distinguish between 4 different samples, the original 3 + 'Heavy Impact'.

When you want to connect the device, you would see it being advertised as 'SilverShield Device 1', the name can be changed inside the prj.conf file. Once the device is connected via Bluetooth, it will say 'Notifications: ENABLED', which will allow you to see the events when they trigger on the GSM Modem. When Bluetooth is not connected, it would print an error line 'BLE Notification failed (err -128)'.


Section 2: Hardware Side (How to connect the devices together)

The IWR6843ISK should have only DIP switch 3 turned to 'on'. 1, 2, 4, 5 and 6 should be 'off'. Connect Type B USB dongle at J5 to power the device. You should see D3, D5, D6 light up first, followed by D4 and finally D7. When D7 turns on, it indicates the radar is running. 

Connect the custom breakout board fittingly named 'IWR6843ISK Breakout Board' to J1 of the IWR6843ISK (J1 is on the bottom side of the IWR6843ISK). Using jumper wires, connect P0.26 of nRF5340 to the pin named 'RS232RX' on the IWR6843ISK Breakout Board, P0.27 of nRF5340 to MSS_LOG of IWR6843ISK Breakout Board and finally connect GND to GND. 

Now that the IWR6843ISK is fully connected to the nRF5340, we must connect the acoustic sensor (Infineon XENSIV™ MEMS Microphone Flex Evaluation Kit) to it. Ensure the microphone is attached to the red breakout board. Pin 1 and 3 of the acoustic sensor is connected to GND of nRF5340, Pin 4 of acoustic sensor is connected to P0.04 of nRF5340, Pin 5 of acoustic sensor is connected to P0.05 of nRF5340 and Pin 6 of acoustic sensor is connected to Vdd of nRF5340. To see outputs, connect a Type B USB to the port called 'IMCU USB'. Ensure the switch named POWER is ON, switch named nRF POWER SOURCE is flipped to Vdd, VEXT -> nRF is off and FLOW CONTROL is set to ON. 

With all this done, the whole system should be functioning. You can go into Visual Studio, plug in the nRF5340 and power the radar, and you should be able to start testing and seeing the data flow. 

References:
@misc{edgeimpulse_dataset_499022,
    title = {Audio Classification - Keyword Spotting},
    author = {Edge Impulse},
    year = {2024},
    url = {https://studio.edgeimpulse.com/public/499022/latest},
    note = {BSD 3-Clause Clear}
}
