package org.usfirst.frc.team449.robot.components;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import com.google.protobuf.InvalidProtocolBufferException;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.usfirst.frc.team449.robot.other.MotionProfileData;
import org.usfirst.frc.team449.robot.other.Waypoint;
import org.zeromq.ZMQ;
import proto.PathOuterClass;
import proto.PathRequestOuterClass;

import java.util.Arrays;

/**
 * The object that requests a motion profile from the Jetson.
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class PathRequester {

    /**
     * The socket for communicating with the Jetson.
     */
    @NotNull
    private final ZMQ.Socket socket;

    /**
     * The path received from the Jetson. Field to avoid garbage collection.
     */
    private PathOuterClass.Path path;

    /**
     * The proto for requesting a motion profile from the JetsopathRequestern. Field to avoid garbage collection.
     */
    private PathRequestOuterClass.PathRequest.Builder pathRequest;

    /**
     * The bytes read from the socket. Field to avoid garbage collection.
     */
    private byte[] output;

    private double wheelbase;

    /**
     * Default constructor.
     *
     * @param address The address of the port on the RIO to open.
     */
    @JsonCreator
    public PathRequester(@NotNull @JsonProperty(required = true) String address,
                         @JsonProperty(required = true) double wheelbase) {
        ZMQ.Context context = ZMQ.context(1);
        socket = context.socket(ZMQ.REQ);
        socket.bind(address);
        this.wheelbase = wheelbase;
    }

    /**
     * Request a motion profile path for a given x, y, and angular displacement.
     *
     * @param waypoints The waypoints to hit on the profile.
     * @param deltaTime The time between setpoints in the profile, in seconds.
     * @param maxVel    The maximum velocity, in units/second.
     * @param maxAccel  The maximum acceleration, in units/(second^2)
     * @param maxJerk   The maximum jerk, in units/(second^3)
     */
    public void requestPath(Waypoint[] waypoints, double deltaTime, double maxVel, double maxAccel, double maxJerk) {
        System.out.println("+++REQUEST PATH&&&&&&&&&&&&&&&++++++++++++++++&&&&&&&&&&&&++++++++++++");

        //Send the request
        pathRequest = PathRequestOuterClass.PathRequest.newBuilder();
        System.out.println(1);
        for (Waypoint waypoint : waypoints) {
            System.out.println(2);
            pathRequest.addX(waypoint.getX());
            System.out.println(3);
            pathRequest.addY(waypoint.getY());
            System.out.println(4);
            pathRequest.addTheta(waypoint.getThetaRadians());
            System.out.println(5);
        }
        System.out.println(6);
        pathRequest.setDt((int) (deltaTime * 1000)); //Convert to milliseconds
        System.out.println(7);
        pathRequest.setMaxVel(maxVel);
        System.out.println(8);
        pathRequest.setMaxAccel(maxAccel);
        System.out.println(9);
        pathRequest.setMaxJerk(maxJerk);
        System.out.println(10);
        pathRequest.setWheelbase(wheelbase);
        System.out.println(11);
        byte[] bytes = pathRequest.build().toByteArray();
        System.out.println(12);
        socket.send(bytes);
        System.out.println("+++!!!!!END REQUEST PATH&&&&&&&&&&&&&&&++++++++++++++++&&&&&&&&&&&&++++++++++++");
    }

    /**
     * Get a motion profile path for a given x, y, and angular displacement.
     *
     * @param inverted      Whether or not to invert the profiles.
     * @param resetPosition Whether or not to reset position when the profile starts.
     * @return Null if the Jetson hasn't replied yet, a list of one profile if theta is 0, or a list of left, right
     * profiles in that order otherwise.
     */
    @Nullable
    public MotionProfileData[] getPath(boolean inverted, boolean resetPosition) {
        System.out.println("++++++GET PATH++++++++++++++++++++++++++++++++");
        //Read from Jetson
        output = socket.recv(ZMQ.NOBLOCK);
        if (output == null) {
            return null;
        }

        //Make these local variables and not fields so that this thread doesn't retain any connection to it.
        MotionProfileData leftMotionProfileData, rightMotionProfileData = null;

        try {
            //Read the response
            path = PathOuterClass.Path.parseFrom(output);
            leftMotionProfileData = new MotionProfileData(path.getPosLeftList(), path.getVelLeftList(),
                    path.getAccelLeftList(), path.getDeltaTime(), inverted, false, resetPosition);
            if (path.getPosRightCount() != 0) {
                rightMotionProfileData = new MotionProfileData(path.getPosRightList(), path.getVelRightList(),
                        path.getAccelRightList(), path.getDeltaTime(), inverted, false, resetPosition);
            }
        } catch (InvalidProtocolBufferException e) {
            System.out.println("Error reading proto!");
            e.printStackTrace();
            return null;
        }

        // code for path gen
        double[][] leftData = leftMotionProfileData.getData();
        double[][] rightData = rightMotionProfileData.getData();
        for (double[] leftDataRow : leftData) {
            System.out.println(Arrays.toString(leftDataRow));
        }
        if (rightMotionProfileData != null) {
            for (double[] rightDataRow : rightData) {
                System.out.println(Arrays.toString(rightDataRow));
            }
        }

        //Return stuff
        if (rightMotionProfileData == null) {
            return new MotionProfileData[]{leftMotionProfileData};
        } else {
            return new MotionProfileData[]{leftMotionProfileData, rightMotionProfileData};
        }
    }
}
