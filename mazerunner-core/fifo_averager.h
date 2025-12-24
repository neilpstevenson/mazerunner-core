#pragma once

// Class to manage a simple FIFO-based averager for filtering encoder counts etc.
template <int fifoLength>
class FifoAverager {

public:
  FifoAverager() : 	m_index(0) {
	  reset();
  }
  
  void reset() {
    for (int i = 0; i < fifoLength; i++) {
        m_fifo[i] = 0;
    }
    m_fifo_total = 0;
  }

  float update(int8_t value) {
    // update the FIFO and moving average
    m_fifo_total -= m_fifo[m_index];
    m_fifo_total += value;
    m_fifo[m_index++] = value;
    if (m_index >= fifoLength) {
      m_index = 0;
    }
	return (float)m_fifo_total / fifoLength;
  }

  float average() {
    return (float)m_fifo_total / fifoLength;
  }

private:
  int8_t m_fifo[fifoLength];
  int m_index;
  int m_fifo_total;
  
};
