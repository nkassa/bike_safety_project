# bike_safety_project

To improve the safety and overall experience of riding a bicycle on college campuses or residential areas with lots of traffic, I have created a bicycle safety enhancing system. By adding specialized modules to bikes, like automated turn signals, I transformed conventional bikes into smart bikes getting rid of outdated conventional techniques. The system uses an ATMEGA328P microcontroller and a custom-built software for user experience and interaction. High Level Design: The rotary encoder (input) is used to measure the speed of the bike and resulting bike speed is displayed on a LCD (output). The system also uses buttons (input) to indicate turning or lane switching. When turn signal buttons are used, turn signal LEDs (output) light up indicating the bike is about to make a turn. When lane switching buttons are used and the sonar sensor readings (inputs) do not detect obstacles, then the user is permitted to turn on the turn signals indicating to others on the road that the bike is changing lanes. But if the sonar sensor senses an obstacle then the turn signals will not light up even if the rider clicks the button because it is not safe to make the maneuver.
