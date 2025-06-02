
#ifndef NRF24L_HPP_
#define NRF24L_HPP_

#include "main.h"

class NRF24L {

public:

	// Result of RX FIFO reading
	typedef enum {
		RX_PIPE0  = (uint8_t)0x00, // Packet received from the PIPE#0
		RX_PIPE1  = (uint8_t)0x01, // Packet received from the PIPE#1
		RX_PIPE2  = (uint8_t)0x02, // Packet received from the PIPE#2
		RX_PIPE3  = (uint8_t)0x03, // Packet received from the PIPE#3
		RX_PIPE4  = (uint8_t)0x04, // Packet received from the PIPE#4
		RX_PIPE5  = (uint8_t)0x05, // Packet received from the PIPE#5
		RX_EMPTY  = (uint8_t)0xff  // The RX FIFO is empty
	} RXResult;

	typedef enum {
		TX_ERROR  = (uint8_t)0x00, // Unknown error
		TX_SUCCESS,                // Packet has been transmitted successfully
		TX_TIMEOUT,                // It was timeout during packet transmit
		TX_MAXRT                   // Transmit failed with maximum auto retransmit count
	} TXResult;

	// Status of the RX FIFO
	typedef enum {
		DATA  = (uint8_t)0x00, // The RX FIFO contains data and available locations
		EMPTY = (uint8_t)0x01, // The RX FIFO is empty
		FULL  = (uint8_t)0x02, // The RX FIFO is full
		ERROR = (uint8_t)0x03  // Impossible state: RX FIFO cannot be empty and full at the same time
	} FifoStatus;

	NRF24L(SPI_HandleTypeDef *spi, GPIO_TypeDef* CEPort, uint16_t CEPin, GPIO_TypeDef* CSNPort, uint16_t CSNPin);
	virtual ~NRF24L();

	void Init(void);
	uint8_t Check(void);
	void InitTX();
	void InitRX(uint8_t *dest, uint8_t len);
	TXResult TransmitPacket(uint8_t *pBuf, uint8_t length);

	void SetPowerMode(uint8_t mode);
	void SetOperationalMode(uint8_t mode);
	void SetRFChannel(uint8_t channel);
	void SetAutoRetr(uint8_t ard, uint8_t arc);
	void SetAddrWidth(uint8_t addr_width);
	void SetAddr(uint8_t pipe, const uint8_t *addr);
	void SetTXPower(uint8_t tx_pwr);
	void SetDataRate(uint8_t data_rate);
	void SetCRCScheme(uint8_t scheme);
	void SetRXPipe(uint8_t pipe, uint8_t aa_state, uint8_t payload_len);
	void ClosePipe(uint8_t pipe);
	void EnableAA(uint8_t pipe);
	void DisableAA(uint8_t pipe);
	void SetDynamicPayloadLength(uint8_t mode);
	void SetPayloadWithAck(uint8_t mode);

	uint8_t GetStatus(void);
	uint8_t GetIRQFlags(void);
	uint8_t GetStatus_RXFIFO(void);
	uint8_t GetStatus_TXFIFO(void);
	uint8_t GetRXSource(void);
	uint8_t GetRetransmitCounters(void);
	uint8_t GetFeatures(void);

	void ResetPLOS(void);
	void FlushTX(void);
	void FlushRX(void);
	void ClearIRQFlags(void);
	void ActivateFeatures(void);
	void WritePayload(uint8_t *pBuf, uint8_t length);
	void WriteAckPayload(RXResult pipe, char *payload, uint8_t length);
	RXResult ReadPayload(uint8_t *pBuf, uint8_t *length);
	RXResult ReadPayloadDpl(uint8_t *pBuf, uint8_t *length);


private:
	SPI_HandleTypeDef *_spi;
	GPIO_TypeDef *_CEPort;
	uint16_t _CEPin;
	GPIO_TypeDef *_CSNPort;
	uint16_t _CSNPin;

	void nRF24_CE_L();
	void nRF24_CE_H();
	void nRF24_CSN_L();
	void nRF24_CSN_H();
	uint8_t nRF24_LL_RW(uint8_t data);

	uint8_t ReadReg(uint8_t reg);
	void WriteReg(uint8_t reg, uint8_t value);
	void ReadMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count);
	void WriteMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count);
	uint8_t GetRxDplPayloadWidth();
	RXResult ReadPayloadGeneric(uint8_t *pBuf, uint8_t *length, uint8_t dpl);

	// nRF24L0 instruction definitions
	#define nRF24_CMD_R_REGISTER       (uint8_t)0x00 // Register read
	#define nRF24_CMD_W_REGISTER       (uint8_t)0x20 // Register write
	#define nRF24_CMD_ACTIVATE         (uint8_t)0x50 // (De)Activates R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK features
	#define nRF24_CMD_R_RX_PL_WID	   (uint8_t)0x60 // Read RX-payload width for the top R_RX_PAYLOAD in the RX FIFO.
	#define nRF24_CMD_R_RX_PAYLOAD     (uint8_t)0x61 // Read RX payload
	#define nRF24_CMD_W_TX_PAYLOAD     (uint8_t)0xA0 // Write TX payload
	#define nRF24_CMD_W_ACK_PAYLOAD    (uint8_t)0xA8 // Write ACK payload
	#define nRF24_CMD_W_TX_PAYLOAD_NOACK (uint8_t) 0xB0//Write TX payload and disable AUTOACK
	#define nRF24_CMD_FLUSH_TX         (uint8_t)0xE1 // Flush TX FIFO
	#define nRF24_CMD_FLUSH_RX         (uint8_t)0xE2 // Flush RX FIFO
	#define nRF24_CMD_REUSE_TX_PL      (uint8_t)0xE3 // Reuse TX payload
	#define nRF24_CMD_LOCK_UNLOCK      (uint8_t)0x50 // Lock/unlock exclusive features
	#define nRF24_CMD_NOP              (uint8_t)0xFF // No operation (used for reading status register)

	// nRF24L0 register definitions
	#define nRF24_REG_CONFIG           (uint8_t)0x00 // Configuration register
	#define nRF24_REG_EN_AA            (uint8_t)0x01 // Enable "Auto acknowledgment"
	#define nRF24_REG_EN_RXADDR        (uint8_t)0x02 // Enable RX addresses
	#define nRF24_REG_SETUP_AW         (uint8_t)0x03 // Setup of address widths
	#define nRF24_REG_SETUP_RETR       (uint8_t)0x04 // Setup of automatic retransmit
	#define nRF24_REG_RF_CH            (uint8_t)0x05 // RF channel
	#define nRF24_REG_RF_SETUP         (uint8_t)0x06 // RF setup register
	#define nRF24_REG_STATUS           (uint8_t)0x07 // Status register
	#define nRF24_REG_OBSERVE_TX       (uint8_t)0x08 // Transmit observe register
	#define nRF24_REG_RPD              (uint8_t)0x09 // Received power detector
	#define nRF24_REG_RX_ADDR_P0       (uint8_t)0x0A // Receive address data pipe 0
	#define nRF24_REG_RX_ADDR_P1       (uint8_t)0x0B // Receive address data pipe 1
	#define nRF24_REG_RX_ADDR_P2       (uint8_t)0x0C // Receive address data pipe 2
	#define nRF24_REG_RX_ADDR_P3       (uint8_t)0x0D // Receive address data pipe 3
	#define nRF24_REG_RX_ADDR_P4       (uint8_t)0x0E // Receive address data pipe 4
	#define nRF24_REG_RX_ADDR_P5       (uint8_t)0x0F // Receive address data pipe 5
	#define nRF24_REG_TX_ADDR          (uint8_t)0x10 // Transmit address
	#define nRF24_REG_RX_PW_P0         (uint8_t)0x11 // Number of bytes in RX payload in data pipe 0
	#define nRF24_REG_RX_PW_P1         (uint8_t)0x12 // Number of bytes in RX payload in data pipe 1
	#define nRF24_REG_RX_PW_P2         (uint8_t)0x13 // Number of bytes in RX payload in data pipe 2
	#define nRF24_REG_RX_PW_P3         (uint8_t)0x14 // Number of bytes in RX payload in data pipe 3
	#define nRF24_REG_RX_PW_P4         (uint8_t)0x15 // Number of bytes in RX payload in data pipe 4
	#define nRF24_REG_RX_PW_P5         (uint8_t)0x16 // Number of bytes in RX payload in data pipe 5
	#define nRF24_REG_FIFO_STATUS      (uint8_t)0x17 // FIFO status register
	#define nRF24_REG_DYNPD            (uint8_t)0x1C // Enable dynamic payload length
	#define nRF24_REG_FEATURE          (uint8_t)0x1D // Feature register

	// Register bits definitions
	#define nRF24_CONFIG_PRIM_RX       (uint8_t)0x01 // PRIM_RX bit in CONFIG register
	#define nRF24_CONFIG_PWR_UP        (uint8_t)0x02 // PWR_UP bit in CONFIG register
	#define nRF24_FEATURE_EN_DYN_ACK   (uint8_t)0x01 // EN_DYN_ACK bit in FEATURE register
	#define nRF24_FEATURE_EN_ACK_PAY   (uint8_t)0x02 // EN_ACK_PAY bit in FEATURE register
	#define nRF24_FEATURE_EN_DPL       (uint8_t)0x04 // EN_DPL bit in FEATURE register
	#define nRF24_FLAG_RX_DR           (uint8_t)0x40 // RX_DR bit (data ready RX FIFO interrupt)
	#define nRF24_FLAG_TX_DS           (uint8_t)0x20 // TX_DS bit (data sent TX FIFO interrupt)
	#define nRF24_FLAG_MAX_RT          (uint8_t)0x10 // MAX_RT bit (maximum number of TX retransmits interrupt)

	// Register masks definitions
	#define nRF24_MASK_REG_MAP         (uint8_t)0x1F // Mask bits[4:0] for CMD_RREG and CMD_WREG commands
	#define nRF24_MASK_CRC             (uint8_t)0x0C // Mask for CRC bits [3:2] in CONFIG register
	#define nRF24_MASK_STATUS_IRQ      (uint8_t)0x70 // Mask for all IRQ bits in STATUS register
	#define nRF24_MASK_RF_PWR          (uint8_t)0x06 // Mask RF_PWR[2:1] bits in RF_SETUP register
	#define nRF24_MASK_RX_P_NO         (uint8_t)0x0E // Mask RX_P_NO[3:1] bits in STATUS register
	#define nRF24_MASK_DATARATE        (uint8_t)0x28 // Mask RD_DR_[5,3] bits in RF_SETUP register
	#define nRF24_MASK_EN_RX           (uint8_t)0x3F // Mask ERX_P[5:0] bits in EN_RXADDR register
	#define nRF24_MASK_RX_PW           (uint8_t)0x3F // Mask [5:0] bits in RX_PW_Px register
	#define nRF24_MASK_RETR_ARD        (uint8_t)0xF0 // Mask for ARD[7:4] bits in SETUP_RETR register
	#define nRF24_MASK_RETR_ARC        (uint8_t)0x0F // Mask for ARC[3:0] bits in SETUP_RETR register
	#define nRF24_MASK_RXFIFO          (uint8_t)0x03 // Mask for RX FIFO status bits [1:0] in FIFO_STATUS register
	#define nRF24_MASK_TXFIFO          (uint8_t)0x30 // Mask for TX FIFO status bits [5:4] in FIFO_STATUS register
	#define nRF24_MASK_PLOS_CNT        (uint8_t)0xF0 // Mask for PLOS_CNT[7:4] bits in OBSERVE_TX register
	#define nRF24_MASK_ARC_CNT         (uint8_t)0x0F // Mask for ARC_CNT[3:0] bits in OBSERVE_TX register

	// Fake address to test transceiver presence (5 bytes long)
	#define nRF24_TEST_ADDR            "nRF24"


	// Retransmit delay
	enum {
		nRF24_ARD_NONE   = (uint8_t)0x00, // Dummy value for case when retransmission is not used
		nRF24_ARD_250us  = (uint8_t)0x00,
		nRF24_ARD_500us  = (uint8_t)0x01,
		nRF24_ARD_750us  = (uint8_t)0x02,
		nRF24_ARD_1000us = (uint8_t)0x03,
		nRF24_ARD_1250us = (uint8_t)0x04,
		nRF24_ARD_1500us = (uint8_t)0x05,
		nRF24_ARD_1750us = (uint8_t)0x06,
		nRF24_ARD_2000us = (uint8_t)0x07,
		nRF24_ARD_2250us = (uint8_t)0x08,
		nRF24_ARD_2500us = (uint8_t)0x09,
		nRF24_ARD_2750us = (uint8_t)0x0A,
		nRF24_ARD_3000us = (uint8_t)0x0B,
		nRF24_ARD_3250us = (uint8_t)0x0C,
		nRF24_ARD_3500us = (uint8_t)0x0D,
		nRF24_ARD_3750us = (uint8_t)0x0E,
		nRF24_ARD_4000us = (uint8_t)0x0F
	};

	// Data rate
	enum {
		nRF24_DR_250kbps = (uint8_t)0x20, // 250kbps data rate
		nRF24_DR_1Mbps   = (uint8_t)0x00, // 1Mbps data rate
		nRF24_DR_2Mbps   = (uint8_t)0x08  // 2Mbps data rate
	};

	// RF output power in TX mode
	enum {
		nRF24_TXPWR_18dBm = (uint8_t)0x00, // -18dBm
		nRF24_TXPWR_12dBm = (uint8_t)0x02, // -12dBm
		nRF24_TXPWR_6dBm  = (uint8_t)0x04, //  -6dBm
		nRF24_TXPWR_0dBm  = (uint8_t)0x06  //   0dBm
	};

	// CRC encoding scheme
	enum {
		nRF24_CRC_off   = (uint8_t)0x00, // CRC disabled
		nRF24_CRC_1byte = (uint8_t)0x08, // 1-byte CRC
		nRF24_CRC_2byte = (uint8_t)0x0c  // 2-byte CRC
	};

	// nRF24L01 power control
	enum {
		nRF24_PWR_UP   = (uint8_t)0x02, // Power up
		nRF24_PWR_DOWN = (uint8_t)0x00  // Power down
	};

	// Transceiver mode
	enum {
		nRF24_MODE_RX = (uint8_t)0x01, // PRX
		nRF24_MODE_TX = (uint8_t)0x00  // PTX
	};

	enum {
		nRF24_DPL_ON = (uint8_t)0x01, // PRX
		nRF24_DPL_OFF = (uint8_t)0x00  // PTX
	} ;

	// Enumeration of RX pipe addresses and TX address
	enum {
		nRF24_PIPE0  = (uint8_t)0x00, // pipe0
		nRF24_PIPE1  = (uint8_t)0x01, // pipe1
		nRF24_PIPE2  = (uint8_t)0x02, // pipe2
		nRF24_PIPE3  = (uint8_t)0x03, // pipe3
		nRF24_PIPE4  = (uint8_t)0x04, // pipe4
		nRF24_PIPE5  = (uint8_t)0x05, // pipe5
		nRF24_PIPETX = (uint8_t)0x06  // TX address (not a pipe in fact)
	};

	// State of auto acknowledgment for specified pipe
	enum {
		nRF24_AA_OFF = (uint8_t)0x00,
		nRF24_AA_ON  = (uint8_t)0x01
	};



	// Addresses of the RX_PW_P# registers
	const uint8_t RX_PW_PIPE[6] = {
			nRF24_REG_RX_PW_P0,
			nRF24_REG_RX_PW_P1,
			nRF24_REG_RX_PW_P2,
			nRF24_REG_RX_PW_P3,
			nRF24_REG_RX_PW_P4,
			nRF24_REG_RX_PW_P5
	};

	// Addresses of the address registers
	const uint8_t ADDR_REGS[7] = {
			nRF24_REG_RX_ADDR_P0,
			nRF24_REG_RX_ADDR_P1,
			nRF24_REG_RX_ADDR_P2,
			nRF24_REG_RX_ADDR_P3,
			nRF24_REG_RX_ADDR_P4,
			nRF24_REG_RX_ADDR_P5,
			nRF24_REG_TX_ADDR
	};


};

#endif /* NRF24L_HPP_ */
