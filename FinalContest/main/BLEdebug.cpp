#include "BLEdebug.h"

// Tạo đối tượng BLEDebug toàn cục
BLEDebug bleDebug;

// Biến tĩnh để truy cập từ callbacks
static bool s_deviceConnected = false;
static String s_receivedCommand = "";
static bool s_newCommandReceived = false;

void BLEDebug::ServerCallbacks::onConnect(BLEServer* pServer) {
    s_deviceConnected = true;
    Serial.println("BLE: Thiết bị đã kết nối");
}

void BLEDebug::ServerCallbacks::onDisconnect(BLEServer* pServer) {
    s_deviceConnected = false;
    Serial.println("BLE: Thiết bị đã ngắt kết nối");
}

void BLEDebug::CharacteristicCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
    // Lấy giá trị dưới dạng chuỗi bytes
    uint8_t* rxData = pCharacteristic->getData();
    size_t rxLength = pCharacteristic->getLength();
    
    if (rxLength > 0) {
        s_receivedCommand = "";
        for (int i = 0; i < rxLength; i++) {
            s_receivedCommand += (char)rxData[i];
        }
        s_newCommandReceived = true;
        
        Serial.print("BLE nhận: ");
        Serial.println(s_receivedCommand);
    }
}

bool BLEDebug::begin(const char* deviceName) {
    // Khởi tạo BLE
    BLEDevice::init(deviceName);
    
    // Tạo BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    // Tạo BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    // Tạo BLE Characteristics
    pCharacteristicRX = pService->createCharacteristic(
                            CHARACTERISTIC_UUID_RX,
                            BLECharacteristic::PROPERTY_WRITE |
                            BLECharacteristic::PROPERTY_NOTIFY
                          );
    
    pCharacteristicTX = pService->createCharacteristic(
                            CHARACTERISTIC_UUID_TX,
                            BLECharacteristic::PROPERTY_READ |
                            BLECharacteristic::PROPERTY_NOTIFY
                          );
                          
    // Thêm descriptor cho notifications
    pCharacteristicRX->addDescriptor(new BLE2902());
    pCharacteristicTX->addDescriptor(new BLE2902());
    
    // Đặt callbacks cho Characteristic RX
    pCharacteristicRX->setCallbacks(new CharacteristicCallbacks());
    
    // Bắt đầu service
    pService->start();
    
    // Bắt đầu quảng cáo
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    
    Serial.println("BLE đã được khởi tạo. Đang chờ client kết nối...");
    
    return true;
}

void BLEDebug::update() {
    // Đồng bộ trạng thái kết nối
    deviceConnected = s_deviceConnected;
    
    // Cập nhật lệnh nhận được
    if (s_newCommandReceived) {
        receivedCommand = s_receivedCommand;
        newCommandReceived = true;
        s_newCommandReceived = false;
    }
    
    // Xử lý kết nối/ngắt kết nối
    if (deviceConnected && !oldDeviceConnected) {
        // Kết nối mới
        oldDeviceConnected = deviceConnected;
    }
    
    if (!deviceConnected && oldDeviceConnected) {
        // Đã ngắt kết nối
        delay(500); // Cho BLE stack thời gian để chuẩn bị cho kết nối mới
        pServer->startAdvertising(); // Khởi động lại quảng cáo
        Serial.println("BLE: Bắt đầu quảng cáo lại");
        oldDeviceConnected = deviceConnected;
    }
}

bool BLEDebug::isConnected() {
    return deviceConnected;
}

void BLEDebug::sendData(const String& data) {
    if (deviceConnected) {
        // Gửi dữ liệu qua BLE
        pCharacteristicTX->setValue(data.c_str());
        pCharacteristicTX->notify();
        // Thêm độ trễ nhỏ để đảm bảo dữ liệu được gửi đi hoàn tất
        delay(3);
    }
}

String BLEDebug::getCommand() {
    String command = "";
    if (newCommandReceived) {
        command = receivedCommand;
        newCommandReceived = false;
    }
    return command;
} 