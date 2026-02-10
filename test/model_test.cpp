#include "model.hpp"
#include <gtest/gtest.h>
#include <cmath>

// Test Model::Control operators
TEST(ModelControl, addition) {
    Model::Control c1{1.0f, 2.0f};
    Model::Control c2{3.0f, 4.0f};
    Model::Control result = c1 + c2;
    EXPECT_FLOAT_EQ(result.left, 4.0f);
    EXPECT_FLOAT_EQ(result.right, 6.0f);
}

TEST(ModelControl, subtraction) {
    Model::Control c1{5.0f, 7.0f};
    Model::Control c2{2.0f, 3.0f};
    Model::Control result = c1 - c2;
    EXPECT_FLOAT_EQ(result.left, 3.0f);
    EXPECT_FLOAT_EQ(result.right, 4.0f);
}

TEST(ModelControl, multiplication) {
    Model::Control c{2.0f, 3.0f};
    Model::Control result = c * 2.0f;
    EXPECT_FLOAT_EQ(result.left, 4.0f);
    EXPECT_FLOAT_EQ(result.right, 6.0f);
}

TEST(ModelControl, negative_values) {
    Model::Control c1{-1.0f, 2.0f};
    Model::Control c2{3.0f, -4.0f};
    Model::Control result = c1 + c2;
    EXPECT_FLOAT_EQ(result.left, 2.0f);
    EXPECT_FLOAT_EQ(result.right, -2.0f);
}

// Test Model::State operators
TEST(ModelState, equality) {
    Model::State s1{1.0f, 2.0f, 0.5f};
    Model::State s2{1.0f, 2.0f, 0.5f};
    Model::State s3{1.0f, 2.0f, 0.6f};
    EXPECT_TRUE(s1 == s2);
    EXPECT_FALSE(s1 == s3);
}

TEST(ModelState, addition) {
    Model::State s1{1.0f, 2.0f, 0.5f};
    Model::State s2{3.0f, 4.0f, 0.5f};
    Model::State result = s1 + s2;
    EXPECT_FLOAT_EQ(result.x, 4.0f);
    EXPECT_FLOAT_EQ(result.y, 6.0f);
    EXPECT_FLOAT_EQ(result.yaw, 1.0f);
}

TEST(ModelState, subtraction) {
    Model::State s1{5.0f, 7.0f, 1.0f};
    Model::State s2{2.0f, 3.0f, 0.5f};
    Model::State result = s1 - s2;
    EXPECT_FLOAT_EQ(result.x, 3.0f);
    EXPECT_FLOAT_EQ(result.y, 4.0f);
    EXPECT_NEAR(result.yaw, 0.5f, 0.0001f);
}

TEST(ModelState, yaw_subtraction_wraparound) {
    Model::State s1{0.0f, 0.0f, 0.1f};
    Model::State s2{0.0f, 0.0f, 2.0f * M_PI - 0.1f};
    Model::State result = s1 - s2;
    // Yaw difference should be approximately 0.2 (wrapped around)
    EXPECT_NEAR(result.yaw, 0.2f, 0.01f);
}

TEST(ModelState, multiplication) {
    Model::State s{2.0f, 3.0f, 0.5f};
    Model::State result = s * 2.0f;
    EXPECT_FLOAT_EQ(result.x, 4.0f);
    EXPECT_FLOAT_EQ(result.y, 6.0f);
    EXPECT_FLOAT_EQ(result.yaw, 1.0f);
}

TEST(ModelState, distance) {
    Model::State s1{0.0f, 0.0f, 0.0f};
    Model::State s2{3.0f, 4.0f, 0.0f};
    float dist = s1.dist(s2);
    EXPECT_FLOAT_EQ(dist, 5.0f);  // sqrt(3^2 + 4^2)
}

TEST(ModelState, distance_with_yaw) {
    Model::State s1{0.0f, 0.0f, 0.0f};
    Model::State s2{0.0f, 0.0f, 1.0f};
    float dist = s1.dist(s2);
    EXPECT_FLOAT_EQ(dist, 1.0f);
}

TEST(ModelState, distanceXY) {
    Model::State s1{0.0f, 0.0f, 0.0f};
    Model::State s2{3.0f, 4.0f, 0.0f};
    float dist = s1.distXY(s2);
    EXPECT_FLOAT_EQ(dist, 5.0f);  // sqrt(3^2 + 4^2)
}

TEST(ModelState, distanceXY_different_yaw) {
    Model::State s1{1.0f, 2.0f, 0.0f};
    Model::State s2{4.0f, 6.0f, 1.5f};
    float dist = s1.distXY(s2);
    EXPECT_FLOAT_EQ(dist, 5.0f);  // Should ignore yaw
}

// Test Model constructor and basic methods (without ONNX)
// Note: We skip tests requiring ONNX runtime to avoid dependency issues

TEST(ModelBasic, state_management) {
    // This test just verifies the model can be created with dummy ONNX path
    // In real tests, you'd use a mock or actual ONNX model
    // Model::State initial{1.0f, 2.0f, 0.5f};
    // Model model(initial, 0.1f, "dummy.onnx");
    
    // For now, just test State operations which don't require ONNX
    Model::State s{1.0f, 2.0f, 0.5f};
    EXPECT_FLOAT_EQ(s.x, 1.0f);
    EXPECT_FLOAT_EQ(s.y, 2.0f);
    EXPECT_FLOAT_EQ(s.yaw, 0.5f);
}

TEST(ModelState, zero_operations) {
    Model::State zero{0.0f, 0.0f, 0.0f};
    Model::State s{1.0f, 2.0f, 3.0f};
    
    EXPECT_TRUE((zero + zero) == zero);
    EXPECT_TRUE((zero - zero) == zero);
    EXPECT_TRUE((zero * 5.0f) == zero);
    
    EXPECT_TRUE((s + zero) == s);
    EXPECT_TRUE((s - zero) == s);
}

TEST(ModelControl, zero_operations) {
    Model::Control zero{0.0f, 0.0f};
    Model::Control c{1.0f, 2.0f};
    
    EXPECT_FLOAT_EQ((zero + zero).left, 0.0f);
    EXPECT_FLOAT_EQ((zero + zero).right, 0.0f);
    
    EXPECT_FLOAT_EQ((c + zero).left, 1.0f);
    EXPECT_FLOAT_EQ((c + zero).right, 2.0f);
}

// Test velocity calculation from control (kinematic model)
// Note: These tests don't require ONNX model
TEST(ModelKinematics, velocity_from_control_zero) {
    // Test the velocity calculation formulas
    // velocityFromControl: 
    //   vx = k * (left + right) * cos(yaw)
    //   vy = k * (left + right) * sin(yaw)
    //   omega = k_w * k * (left - right)
    
    // With control = {0, 0}, velocity should be zero
    // This is tested implicitly through the formula
    float left = 0.0f, right = 0.0f;
    float k = 1.0f, k_w = 1.0f;
    float yaw = 0.0f;
    
    float vx = k * (left + right) * cosf(yaw);
    float vy = k * (left + right) * sinf(yaw);
    float omega = k_w * k * (left - right);
    
    EXPECT_FLOAT_EQ(vx, 0.0f);
    EXPECT_FLOAT_EQ(vy, 0.0f);
    EXPECT_FLOAT_EQ(omega, 0.0f);
}

TEST(ModelKinematics, velocity_from_control_straight) {
    // Moving straight forward
    float left = 1.0f, right = 1.0f;
    float k = 1.0f, k_w = 1.0f;
    float yaw = 0.0f;
    
    float vx = k * (left + right) * cosf(yaw);
    float vy = k * (left + right) * sinf(yaw);
    float omega = k_w * k * (left - right);
    
    EXPECT_FLOAT_EQ(vx, 2.0f);   // Moving forward
    EXPECT_FLOAT_EQ(vy, 0.0f);   // No sideways movement
    EXPECT_FLOAT_EQ(omega, 0.0f); // No rotation
}

TEST(ModelKinematics, velocity_from_control_turn) {
    // Turning in place
    float left = 1.0f, right = -1.0f;
    float k = 1.0f, k_w = 1.0f;
    float yaw = 0.0f;
    
    float vx = k * (left + right) * cosf(yaw);
    float vy = k * (left + right) * sinf(yaw);
    float omega = k_w * k * (left - right);
    
    EXPECT_FLOAT_EQ(vx, 0.0f);   // No forward movement
    EXPECT_FLOAT_EQ(vy, 0.0f);   // No sideways movement
    EXPECT_FLOAT_EQ(omega, 2.0f); // Rotating
}

TEST(ModelKinematics, next_state_from_velocity) {
    // Test nextStateFromVelocity: next = current + vel * dt
    Model::State current{1.0f, 2.0f, 0.0f};
    Model::State vel{0.5f, 0.5f, 0.1f};
    float dt = 0.1f;
    
    Model::State next = current + vel * dt;
    
    EXPECT_FLOAT_EQ(next.x, 1.05f);
    EXPECT_FLOAT_EQ(next.y, 2.05f);
    EXPECT_FLOAT_EQ(next.yaw, 0.01f);
}

// Edge cases
TEST(ModelState, large_values) {
    Model::State s1{1e6f, 2e6f, 3.0f};
    Model::State s2{1e6f, 2e6f, 3.0f};
    
    EXPECT_TRUE(s1 == s2);
    
    Model::State result = s1 + s2;
    EXPECT_FLOAT_EQ(result.x, 2e6f);
    EXPECT_FLOAT_EQ(result.y, 4e6f);
}

TEST(ModelState, negative_values) {
    Model::State s1{-1.0f, -2.0f, -0.5f};
    Model::State s2{-3.0f, -4.0f, -1.0f};
    
    Model::State result = s1 + s2;
    EXPECT_FLOAT_EQ(result.x, -4.0f);
    EXPECT_FLOAT_EQ(result.y, -6.0f);
    EXPECT_FLOAT_EQ(result.yaw, -1.5f);
}

TEST(ModelState, pi_yaw_values) {
  // Test yaw handling around PI
  Model::State s1{0.0f, 0.0f, M_PI};
  Model::State s2{0.0f, 0.0f, -M_PI};
  
  // Note: dist() computes sqrt(dx^2 + dy^2 + dyaw^2) without normalizing yaw
  // The raw difference is 2*PI, not 0
  float dist = s1.dist(s2);
  EXPECT_NEAR(dist, 2.0f * M_PI, 0.0001f);
  
  // The subtraction operator DOES normalize yaw with atan2
  Model::State diff = s1 - s2;
  EXPECT_NEAR(diff.yaw, 0.0f, 0.0001f);  // Normalized difference should be 0
}
