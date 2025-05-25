#include "BluetoothSerial.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

BluetoothSerial serialBT;

// ========== CẤU HÌNH CHÂN ĐỘNG CƠ ==========
const int ENA = 14;    // PWM motor phải trước
const int IN1 = 27;    // Chiều motor phải trước
const int IN2 = 26;    // Chiều motor phải trước
const int ENB = 32;    // PWM motor phải sau (và trái)
const int IN3 = 25;    // Chiều motor phải sau (và trái trước)
const int IN4 = 33;    // Chiều motor phải sau (và trái sau)

// ========== CẤU HÌNH CẢM BIẾN SIÊU ÂM ==========
const int trigPinCenter = 19;
const int echoPinCenter = 21;
const int trigPinLeft = 18;
const int echoPinLeft = 5;
const int trigPinRight = 17;
const int echoPinRight = 16;

// ========== CẢM BIẾN ĐƯỜNG LINE ==========
const int lineSensorLeft = 13;
const int lineSensorRight = 22;
const int lineSensorRearLeft = 35;   // Cảm biến đuôi trái
const int lineSensorRearRight = 4;   // Cảm biến đuôi phải

// ========== BIẾN TOÀN CỤC ==========
char incoming_signal;
int Speed = 200;
volatile bool sumoMode = false;
volatile bool bluetoothConnected = false;

// ========== TRẠNG THÁI SUMO ==========
enum SumoState { SEARCHING, ATTACKING, AVOIDING, DEFENDING };
SumoState currentState = SEARCHING;

// ========== HƯỚNG ĐỐI THỦ ==========
enum Direction { FRONT, LEFT, RIGHT, NONE };

// ========== DỮ LIỆU CẢM BIẾN ==========
struct SensorData {
  long leftDistance;
  long centerDistance;
  long rightDistance;
};

volatile bool leftLineDetected = false;
volatile bool rightLineDetected = false;
volatile bool rearLeftLineDetected = false; 
volatile bool rearRightLineDetected = false;

// ========== SEMAPHORE VÀ TASK HANDLES ==========
SemaphoreHandle_t sensorSemaphore;
TaskHandle_t bluetoothTaskHandle = NULL;
TaskHandle_t sumoTaskHandle = NULL;

// ========== KHAI BÁO HÀM ==========
Direction detectOpponent(SensorData distances);
void setMotor(int EN, int IN1, int IN2, int speed, bool reverse);
void stop();
void forward();
void backward();
void turnLeft();
void turnRight();
void turnLeft45();
void turnRight45();
SensorData getDistances();
void updateLineSensors();
Direction findShortestDistanceDirection(SensorData distances);

// ========== HÀM ĐIỀU KHIỂN ĐỘNG CƠ ==========
void setMotor(int EN, int IN1, int IN2, int speed, bool reverse) {
  analogWrite(EN, speed);
  digitalWrite(IN1, reverse ? HIGH : LOW);
  digitalWrite(IN2, reverse ? LOW : HIGH);
}

void stop() {
  setMotor(ENA, IN1, IN2, 0, false);
  setMotor(ENB, IN3, IN4, 0, false);
}

void forward() {
  setMotor(ENA, IN1, IN2, Speed, true);
  setMotor(ENB, IN3, IN4, Speed, true);
}

void backward() {
  setMotor(ENA, IN1, IN2, Speed, false);
  setMotor(ENB, IN3, IN4, Speed, false);
}

void turnLeft() {
  setMotor(ENA, IN1, IN2, Speed, false);
  setMotor(ENB, IN3, IN4, Speed, true);
}

void turnRight() {
  setMotor(ENA, IN1, IN2, Speed, true);
  setMotor(ENB, IN3, IN4, Speed, false);
}

void turnLeft45() {
  if (!sumoMode) return; // Chỉ hoạt động ở chế độ Sumo
  turnLeft();
  vTaskDelay(200 / portTICK_PERIOD_MS); // Thời gian cần điều chỉnh để xoay đúng 45 độ
  stop();
}

void turnRight45() {
  if (!sumoMode) return; // Chỉ hoạt động ở chế độ Sumo
  turnRight();
  vTaskDelay(200 / portTICK_PERIOD_MS); // Thời gian cần điều chỉnh để xoay đúng 45 độ
  stop();
}

// ========== HÀM CẢM BIẾN ==========
SensorData getDistances() {
  SensorData distances;
  
  // Đo cảm biến trái
  digitalWrite(trigPinLeft, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinLeft, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinLeft, LOW);
  distances.leftDistance = pulseIn(echoPinLeft, HIGH, 10000) * 0.034 / 2;
  if (distances.leftDistance <= 0) distances.leftDistance = 999;
  
  // Đo cảm biến giữa
  digitalWrite(trigPinCenter, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinCenter, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinCenter, LOW);
  distances.centerDistance = pulseIn(echoPinCenter, HIGH, 10000) * 0.034 / 2;
  if (distances.centerDistance <= 0) distances.centerDistance = 999;
  
  // Đo cảm biến phải
  digitalWrite(trigPinRight, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinRight, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinRight, LOW);
  distances.rightDistance = pulseIn(echoPinRight, HIGH, 10000) * 0.034 / 2;
  if (distances.rightDistance <= 0) distances.rightDistance = 999;

  return distances;
}

Direction detectOpponent(SensorData distances) {
  const int DETECTION_THRESHOLD = 60; // cm
  
  long minDistance = min(min(distances.leftDistance, distances.centerDistance), distances.rightDistance);
  
  if (minDistance > DETECTION_THRESHOLD) return NONE;
  
  if (minDistance == distances.centerDistance) return FRONT;
  else if (minDistance == distances.leftDistance) return LEFT;
  else return RIGHT;
}

Direction findShortestDistanceDirection(SensorData distances) {
  long minDistance = min(min(distances.leftDistance, distances.centerDistance), distances.rightDistance);
  
  if (minDistance == distances.centerDistance) return FRONT;
  else if (minDistance == distances.leftDistance) return LEFT;
  else return RIGHT;
}

void updateLineSensors() {
  if (xSemaphoreTake(sensorSemaphore, portMAX_DELAY) == pdTRUE) {
    leftLineDetected = !digitalRead(lineSensorLeft);
    rightLineDetected = !digitalRead(lineSensorRight);
    rearLeftLineDetected = !digitalRead(lineSensorRearLeft);
    rearRightLineDetected = !digitalRead(lineSensorRearRight);
    xSemaphoreGive(sensorSemaphore);
  }
}

// ========== TASK XỬ LÝ SUMO ==========
void sumoTask(void *pvParameters) {
  TickType_t lastStateChange = xTaskGetTickCount();
  Direction lastOpponentDirection = NONE;
  Direction targetDirection = NONE;
  bool isTurning = false;
  TickType_t turnStartTime = 0;
  bool lineAvoidanceComplete = true;
  TickType_t forwardStartTime = 0;
  bool isForward = false;

  while (1) {
    if (!sumoMode) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }
    
    // Đọc cảm biến
    SensorData distances = getDistances();
    Direction opponentDirection = detectOpponent(distances);
    updateLineSensors();
    
    // Debug thông tin
    Serial.print("L:");
    Serial.print(distances.leftDistance);
    Serial.print("cm C:");
    Serial.print(distances.centerDistance);
    Serial.print("cm R:");
    Serial.print(distances.rightDistance);
    Serial.print("cm Dir:");
    Serial.print(opponentDirection);
    Serial.print(" Line L:");
    Serial.print(leftLineDetected);
    Serial.print(" R:");
    Serial.println(rightLineDetected);
    
    // Xử lý đường biên (ưu tiên cao nhất)
    if ((leftLineDetected || rightLineDetected || rearLeftLineDetected || rearRightLineDetected) && lineAvoidanceComplete) {
      currentState = AVOIDING;
      lineAvoidanceComplete = false;
      isForward = false; // Reset trạng thái đi thẳng
      Serial.println("Bắt đầu xử lý tránh line");
    }
    
    switch(currentState) {
      case SEARCHING:
        if (opponentDirection != NONE) {
          lastOpponentDirection = opponentDirection;
          currentState = ATTACKING;
          Speed = 255;
          isForward = false; // Reset trạng thái đi thẳng
          lastStateChange = xTaskGetTickCount();
        } else {
          // Logic tìm kiếm mới: đi thẳng, kiểm tra biên, quyết định quay
          if (!isForward) {
            forward();
            forwardStartTime = xTaskGetTickCount();
            isForward = true;
            Serial.println("SEARCHING: Đi thẳng");
          }
          
          // Kiểm tra sau khi đi thẳng 500ms
          if (isForward && (xTaskGetTickCount() - forwardStartTime >= 500 / portTICK_PERIOD_MS)) {
            updateLineSensors();
            
            if (leftLineDetected || rearLeftLineDetected) {
              Serial.println("SEARCHING: Phát hiện biên trái - Quay phải");
              turnRight();
              vTaskDelay(400 / portTICK_PERIOD_MS); // Quay trong 400ms
              isForward = false; // Reset để đi thẳng lại
            } else if (rightLineDetected || rearRightLineDetected) {
              Serial.println("SEARCHING: Phát hiện biên phải - Quay trái");
              turnLeft();
              vTaskDelay(400 / portTICK_PERIOD_MS); // Quay trong 400ms
              isForward = false; // Reset để đi thẳng lại
            } else {
              // Không phát hiện biên, tiếp tục đi thẳng
              Serial.println("SEARCHING: Không phát hiện biên - Tiếp tục đi thẳng");
              isForward = false; // Reset để lặp lại chu kỳ đi thẳng
            }
          }
        }
        break;
        
      case ATTACKING:
        if (opponentDirection != NONE) {
          if (opponentDirection == LEFT && targetDirection == NONE) {
            targetDirection = findShortestDistanceDirection(distances);
            if (targetDirection != FRONT) {
              isTurning = true;
              turnStartTime = xTaskGetTickCount();
            }
          }
          if (isTurning) {
            if (targetDirection == LEFT) {
              turnLeft();
            } else if (targetDirection == RIGHT) {
              turnRight();
            }
            if (xTaskGetTickCount() - turnStartTime >= 200 / portTICK_PERIOD_MS) {
              isTurning = false;
              targetDirection = NONE;
              forward();
            }
          } else {
            if (targetDirection != NONE && targetDirection == FRONT) {
              forward();
            } else if (opponentDirection == FRONT) {
              forward();
            } else if (opponentDirection == RIGHT) {
              turnRight();
            }
          }
          lastOpponentDirection = opponentDirection;
          lastStateChange = xTaskGetTickCount();
        } else if (xTaskGetTickCount() - lastStateChange > 500 / portTICK_PERIOD_MS) {
          currentState = SEARCHING;
          Speed = 200;
          targetDirection = NONE;
          isTurning = false;
          isForward = false; // Reset trạng thái đi thẳng
        }
        break;
        
      case AVOIDING:
        if (leftLineDetected && rightLineDetected && rearLeftLineDetected && rearRightLineDetected) {
          Serial.println("Tất cả cảm biến phát hiện line - Xử lý góc");
          backward();
          vTaskDelay(400 / portTICK_PERIOD_MS);
          turnRight();
          vTaskDelay(800 / portTICK_PERIOD_MS);
        } 
        else if (leftLineDetected && rightLineDetected) {
          Serial.println("Cả trái và phải phát hiện line");
          backward();
          vTaskDelay(300 / portTICK_PERIOD_MS);
          turnRight();
          vTaskDelay(600 / portTICK_PERIOD_MS);
        }
        else if (rearLeftLineDetected && rearRightLineDetected) {
          Serial.println("Cả 2 cảm biến đuôi phát hiện line - Tiến thẳng");
          forward();
          vTaskDelay(300 / portTICK_PERIOD_MS);
        }
        else if (rearLeftLineDetected) {
          Serial.println("Đuôi trái phát hiện line - Quay phải 45 độ");
          turnRight45();
          forward();
          vTaskDelay(300 / portTICK_PERIOD_MS);
        }
        else if (rearRightLineDetected) {
          Serial.println("Đuôi phải phát hiện line - Quay trái 45 độ");
          turnLeft45();
          forward();
          vTaskDelay(300 / portTICK_PERIOD_MS);
        }
        else if (leftLineDetected) {
          Serial.println("Chỉ trái phát hiện line");
          backward();
          vTaskDelay(200 / portTICK_PERIOD_MS);
          turnRight();
          vTaskDelay(400 / portTICK_PERIOD_MS);
        }
        else if (rightLineDetected) {
          Serial.println("Chỉ phải phát hiện line");
          backward();
          vTaskDelay(200 / portTICK_PERIOD_MS);
          turnLeft();
          vTaskDelay(400 / portTICK_PERIOD_MS);
        }
        
        stop();
        updateLineSensors();
        
        if (!leftLineDetected && !rightLineDetected && !rearLeftLineDetected && !rearRightLineDetected) {
          lineAvoidanceComplete = true;
          currentState = SEARCHING;
          isForward = false; // Reset trạng thái đi thẳng
          Serial.println("Hoàn thành xử lý tránh line");
        }
        break;

      default:
        break;
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ========== TASK BLUETOOTH ==========
void bluetoothTask(void *pvParameters) {
  while (1) {
    if (serialBT.available()) {
      incoming_signal = serialBT.read();
      
      if (incoming_signal == 'K') {
        sumoMode = !sumoMode;
        if (sumoMode) {
          currentState = SEARCHING;
          Speed = 200;
          serialBT.println("SUMO Mode: ON");
          Speed = 20; // Thay đổi tốc độ mặc định từ 200 thành 10
        } else {
          stop();
          serialBT.println("SUMO Mode: OFF");
        }
      }
      
      if (!sumoMode) {
        switch(incoming_signal) {
          case '0'...'9': Speed = 100 + (incoming_signal - '0') * 15; break;
          case 'q': Speed = 255; break;
          case 'F': 
            Serial.println("Bluetooth: Forward");
            forward(); 
            break;
          case 'B': 
            Serial.println("Bluetooth: Backward");
            backward(); 
            break;
          case 'L': 
            Serial.println("Bluetooth: Turn Left (continuous)");
            turnLeft(); 
            break;
          case 'R': 
            Serial.println("Bluetooth: Turn Right (continuous)");
            turnRight(); 
            break;
          case 'S': 
            Serial.println("Bluetooth: Stop");
            stop(); 
            break;
          default: break;
        }
      }
    }
    
    // Kiểm tra kết nối Bluetooth
    static bool lastConnected = false;
    bool connected = serialBT.connected();
    if (connected != lastConnected) {
      if (connected) {
        bluetoothConnected = true;
        serialBT.println("Connected!");
      } else {
        bluetoothConnected = false;
        sumoMode = false;
        stop();
      }
      lastConnected = connected;
    }
    
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  serialBT.begin("SumoRobot");
  
  // Cấu hình chân động cơ
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Cấu hình cảm biến siêu âm
  pinMode(trigPinCenter, OUTPUT);
  pinMode(echoPinCenter, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  
  // Cấu hình cảm biến line
  pinMode(lineSensorLeft, INPUT_PULLUP);
  pinMode(lineSensorRight, INPUT_PULLUP);
  pinMode(lineSensorRearLeft, INPUT_PULLUP);
  pinMode(lineSensorRearRight, INPUT_PULLUP);
  
  stop();
  
  // Tạo semaphore và task
  sensorSemaphore = xSemaphoreCreateMutex();
  
  xTaskCreate(sumoTask, "Sumo", 4096, NULL, 2, &sumoTaskHandle);
  xTaskCreate(bluetoothTask, "BT", 4096, NULL, 1, &bluetoothTaskHandle);
}

// ========== LOOP ==========
void loop() {
  vTaskDelete(NULL);
}