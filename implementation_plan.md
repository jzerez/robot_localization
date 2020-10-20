## Implementation Plan
### Key Steps
1. Random Sampling of `n` points throughout entire map
2. Apply motion transform to the `n` sampled points based on data from `\odom`
3. Look at sensor data
4. Calculate the probability that each point matches the sensor data recorded
5. Re-sample points based on probability (probability of resampling is equal to: 1 - probability of location)
  * Consider other criteria for which points require re-sampling (re-sample least x% probable? re-sample all points whose probability is less than y%? )
  * Determine criteria for the location of re-sampled points. Does it follow some distribution around the likely points? What is the variance of that distribution?
6. Update robot pose
  * Determine criteria for doing this (pick the highest probability point? Randomly sample from probability field?)
  * Stretch goal: implement some means

### Testing:
1. Make a function that spawns `n` particles following some distribution
  * Plot them in RVIZ and validate by eye
2. Make a function transforms the points based on odom information + some noise
  * Plot them in RVIZ and validate by eye
3. Make a function that takes "would be" lidar scans at the location and orientation of each of the `n` particles
  * Teleport NEATO to the same location and see if the lidar points match up in RVIZ
4. Make a function that calculates the probability associated with each particle
  * Plot them in RVIZ with color or size proportional to probability
  * Qualitatively verify that probability seems reasonable
5. Make a function that updates the robot's pose according to the weighted particles
  * Plot in RVIZ and make sure new robot pose is in the right spot and that a reasonable spot is chosen
