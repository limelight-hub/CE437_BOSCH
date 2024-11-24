# **CAN Diagnostic Communication Practice**

## **Overview**
This project demonstrates the implementation of diagnostic communication services using the CAN protocol. It focuses on key UDS services ($22, $27, $2E) and the CAN_TP layer, commonly used in automotive ECUs.

---

## **Features**
- **UDS Diagnostic Services**:
  - `$22 - ReadDataByIdentifier`: Retrieve data from the ECU.
  - `$27 - SecurityAccess`: Enable secured operations via seed-key exchange.
  - `$2E - WriteDataByIdentifier`: Update ECU configuration data.
- Implements CAN_TP for data transfer beyond 8 bytes.
- Familiarization with ISO standards: ISO11898 and ISO14229.

---

## **System Architecture**
The system consists of:
1. **Tester**: Sends diagnostic requests.
2. **ECU**: Responds to diagnostic commands.
3. **CAN Bus**: Facilitates communication between the Tester and ECU.

**Layered Communication Structure**:
- **Physical Layer**: Based on ISO11898-2/5.
- **Data Link Layer**: Compliant with ISO11898-1.
- **Network Layer**: Implements addressing and CAN IDs.
- **Application Layer**: Executes diagnostic services.

---

## **Practice Objectives**
1. Implement `$22` to read data from the ECU.
2. Use `$2E` to write data into the ECU.
3. Securely access ECU services using `$27`.

---

## **Setup**
### **Hardware Requirements**
- CAN-enabled microcontroller or CAN transceiver.
- Oscilloscope (optional, for debugging CAN frames).

### **Software Requirements**
- C/C++ compiler or a development environment compatible with your microcontroller.
- CAN protocol libraries.

### **Dependencies**
- [CAN_TP Protocol Library](#) (if using a pre-built library for transport protocol).
- Diagnostic UDS protocol documentation (ISO14229).

---

## **How to Run**
1. **Configure the CAN Bus**:
   - Ensure the communication speed matches between the Tester and ECU.
   - Use 11-bit CAN identifiers.

2. **Compile and Upload Code**:
   - Implement the provided services (`$22`, `$27`, `$2E`) on the ECU firmware.
   - Configure the Tester to send diagnostic requests.

3. **Run Diagnostic Tests**:
   - Use the Tester to send `$22`, `$27`, and `$2E` requests.
   - Observe responses from the ECU for validation.

4. **Verify Outputs**:
   - Validate responses using a CAN analyzer or oscilloscope.
   - Check logs for correct message format and timing.

---

## **Expected Results**
- Successful retrieval of ECU data using `$22`.
- Controlled access to secure ECU operations via `$27`.
- Updates to ECU configuration data using `$2E`.

---

## **Evaluation Criteria**
- **Functionality**: Services perform as per defined requirements.
- **Message Formatting**: Compliance with UDS and CAN standards.
- **Performance**: Efficient message handling and response times.
- **Code Quality**: Clean and modular implementation.

---

## **Troubleshooting**
1. **No Response from ECU**:
   - Check CAN Bus connections.
   - Ensure matching CAN IDs and speeds.
2. **Invalid Response Codes**:
   - Verify request formatting.
   - Check for missing parameters in the request message.
3. **Security Access Denied**:
   - Ensure correct seed-key algorithm implementation.

---

## **References**
- ISO11898: CAN Protocol Standards.
- ISO14229: UDS Protocol Standards.
- Training Materials and Documentation.

---

## **Contributors**
- [Tran Dong Truc Lam](#)

