// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: PMotion.proto

package messages;

public interface MotionRequestOrBuilder extends
    // @@protoc_insertion_point(interface_extends:messages.MotionRequest)
    com.google.protobuf.MessageOrBuilder {

  /**
   * <code>optional bool stop_body = 1;</code>
   */
  boolean hasStopBody();
  /**
   * <code>optional bool stop_body = 1;</code>
   */
  boolean getStopBody();

  /**
   * <code>optional bool stop_head = 2;</code>
   */
  boolean hasStopHead();
  /**
   * <code>optional bool stop_head = 2;</code>
   */
  boolean getStopHead();

  /**
   * <code>optional bool reset_odometry = 3;</code>
   */
  boolean hasResetOdometry();
  /**
   * <code>optional bool reset_odometry = 3;</code>
   */
  boolean getResetOdometry();

  /**
   * <code>optional bool remove_stiffness = 4;</code>
   */
  boolean hasRemoveStiffness();
  /**
   * <code>optional bool remove_stiffness = 4;</code>
   */
  boolean getRemoveStiffness();

  /**
   * <code>optional bool enable_stiffness = 5;</code>
   */
  boolean hasEnableStiffness();
  /**
   * <code>optional bool enable_stiffness = 5;</code>
   */
  boolean getEnableStiffness();

  /**
   * <code>optional bool reset_providers = 6;</code>
   */
  boolean hasResetProviders();
  /**
   * <code>optional bool reset_providers = 6;</code>
   */
  boolean getResetProviders();

  /**
   * <code>optional int64 timestamp = 7;</code>
   */
  boolean hasTimestamp();
  /**
   * <code>optional int64 timestamp = 7;</code>
   */
  long getTimestamp();
}
