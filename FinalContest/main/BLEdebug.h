#ifndef BLE_DEBUG_H
#define BLE_DEBUG_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// UUID đã được cấu hình sẵn
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a9" // For sending back PID values

// Class BLEDebug để quản lý kết nối BLE và giao tiếp
class BLEDebug {
public:
    // Khởi tạo BLE với tên thiết bị
    bool begin(const char* deviceName);
    
    // Cập nhật trạng thái kết nối
    void update();
    
    // Kiểm tra xem có thiết bị đang kết nối không
    bool isConnected();
    
    // Gửi thông tin debug qua BLE
    void sendData(const String& data);
    
    // Kiểm tra và trả về lệnh nhận được qua BLE
    String getCommand();
    
    // Class xử lý callbacks khi có kết nối/ngắt kết nối BLE
    class ServerCallbacks : public BLEServerCallbacks {
        void onConnect(BLEServer* pServer);
        void onDisconnect(BLEServer* pServer);
    };
    
    // Class xử lý callbacks khi nhận được dữ liệu từ thiết bị khác
    class CharacteristicCallbacks : public BLECharacteristicCallbacks {
        void onWrite(BLECharacteristic *pCharacteristic);
    };
    
private:
    BLEServer *pServer = NULL;
    BLECharacteristic *pCharacteristicRX = NULL;
    BLECharacteristic *pCharacteristicTX = NULL;
    bool deviceConnected = false;
    bool oldDeviceConnected = false;
    String receivedCommand = "";
    bool newCommandReceived = false;
};

// Khai báo biến toàn cục để sử dụng trong các file khác
extern BLEDebug bleDebug;

#endif // BLE_DEBUG_H 