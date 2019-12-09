#Smooths a value while taking its derivative with respect to time.
smoothDerivative <- function(value, timeMillis, n){
  smoothed <- (value[(n+1):length(value)] - value[1:(length(value)-n)])/((timeMillis[(n+1):length(timeMillis)] - timeMillis[1:(length(timeMillis)-n)])/1000);
  return(c(rep(0, ceiling(n/2)), smoothed, rep(0, floor(n/2))));
}

characterizeDriveRamp <- function(velFiles, smoothing = 3){
  combinedLeftVoltage = c()
  combinedRightVoltage = c()
  combinedLeftVel = c()
  combinedRightVel = c()
  combinedLeftAccel = c()
  combinedRightAccel = c()
  for (velFile in velFiles){
    vel <- read.csv(velFile)
    goodVel <- subset(vel, abs(left.velocity) > 0.1 & abs(left.voltage) > 0.1 & abs(right.velocity) > 0.1 & right.voltage!=0)
    goodVel <- goodVel[1:(length(goodVel$time) - 1), ]
    goodVel$left_accel <- smoothDerivative(goodVel$left.velocity, goodVel$time, smoothing)
    goodVel$right_accel <- smoothDerivative(goodVel$right.velocity, goodVel$time, smoothing)
    goodVel <- subset(goodVel, left_accel != 0 & right_accel != 0)
    plot(goodVel$left.voltage, goodVel$left.velocity)
    plot(goodVel$time, goodVel$left_accel)
    combinedLeftVoltage <- c(combinedLeftVoltage, goodVel$left.voltage)
    combinedRightVoltage <- c(combinedRightVoltage, goodVel$right.voltage)
    combinedLeftVel <- c(combinedLeftVel, goodVel$left.velocity)
    combinedRightVel <- c(combinedRightVel, goodVel$right.velocity)
    combinedLeftAccel <- c(combinedLeftAccel, goodVel$left_accel)
    combinedRightAccel <- c(combinedRightAccel, goodVel$right_accel)
  }
  leftModel <- lm(combinedLeftVoltage~combinedLeftVel+combinedLeftAccel)
  rightModel <- lm(combinedRightVoltage~combinedRightVel+combinedRightAccel)
  print(summary(leftModel))
  print(summary(rightModel))
}

characterizeMotor <- function(velFile, accelFile, smoothing = 2){
  vel <- read.csv(velFile)
  accel <- read.csv(accelFile)
  goodVel <- subset(vel, abs(frontRight.velocity) > 0.1 & abs(frontRight.voltage) > 0.1)
  goodVel <- goodVel[1:(length(goodVel$time) - 1), ]
  goodVel$accel <- smoothDerivative(goodVel$frontRight.velocity, goodVel$time, smoothing)
  accel$accel <- smoothDerivative(accel$frontRight.velocity, accel$time, smoothing)
  print(length(accel$time))
  goodAccel <- subset(accel, frontRight.voltage != 0)
  print(length(goodAccel$time))
  goodAccel <- goodAccel[1:(length(goodAccel$time) - 2),]
  goodAccelLeft <- goodAccel[(which.max(abs(goodAccel$accel))+1):length(goodAccel$time),]
  combinedLeftVoltage <- c(goodVel$frontRight.voltage, goodAccelLeft$frontRight.voltage)
  combinedLeftVel <- c(goodVel$frontRight.velocity, goodAccelLeft$frontRight.velocity)
  combinedLeftAccel <- c(goodVel$accel, goodAccelLeft$accel)
  plot(goodAccelLeft$time, goodAccelLeft$accel)
  plot(goodVel$time, goodVel$frontRight.voltage)
  plot(goodVel$frontRight.voltage, goodVel$frontRight.velocity)
  leftModel <- lm(combinedLeftVoltage~combinedLeftVel+combinedLeftAccel)
  print(summary(leftModel))
}

characterizeDrive <- function(velFile, accelFile, smoothing = 2){
  vel <- read.csv(velFile)
  accel <- read.csv(accelFile)
  goodVel <- subset(vel, abs(left.velocity) > 0.1 & abs(left.voltage) > 0.1 & abs(right.velocity) > 0.1 & right.voltage!=0)
  goodVel <- goodVel[1:(length(goodVel$time) - 1), ]
  goodVel$left_accel <- smoothDerivative(goodVel$left.velocity, goodVel$time, smoothing)
  goodVel$right_accel <- smoothDerivative(goodVel$right.velocity, goodVel$time, smoothing)
  accel$left_accel <- smoothDerivative(accel$left.velocity, accel$time, smoothing)
  accel$right_accel <- smoothDerivative(accel$right.velocity, accel$time, smoothing)
  print(length(accel$time))
  goodAccel <- subset(accel, left.voltage != 0 & right.voltage != 0)
  print(length(goodAccel$time))
  goodAccel <- goodAccel[1:(length(goodAccel$time) - 2),]
  goodAccelLeft <- goodAccel[(which.max(abs(goodAccel$left_accel))+1):length(goodAccel$time),]
  goodAccelRight <- goodAccel[(which.max(abs(goodAccel$right_accel))+1):length(goodAccel$time),]
  combinedLeftVoltage <- c(goodVel$left.voltage, goodAccelLeft$left.voltage)
  combinedRightVoltage <- c(goodVel$right.voltage, goodAccelRight$right.voltage)
  combinedLeftVel <- c(goodVel$left.velocity, goodAccelLeft$left.velocity)
  combinedRightVel <- c(goodVel$right.velocity, goodAccelRight$right.velocity)
  combinedLeftAccel <- c(goodVel$left_accel, goodAccelLeft$left_accel)
  combinedRightAccel <- c(goodVel$right_accel, goodAccelRight$right_accel)
  plot(goodAccelLeft$time, goodAccelLeft$left_accel)
  plot(goodAccelRight$time, goodAccelRight$right_accel)
  plot(goodVel$time, goodVel$right.voltage)
  plot(goodVel$right.voltage, goodVel$right.velocity)
  leftModel <- lm(combinedLeftVoltage~combinedLeftVel+combinedLeftAccel)
  rightModel <- lm(combinedRightVoltage~combinedRightVel+combinedRightAccel)
  print(summary(leftModel))
  print(summary(rightModel))
}