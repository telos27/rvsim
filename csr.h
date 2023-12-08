#pragma once

uint32_t read_CSR(uint32_t CSR_no);
uint32_t write_CSR(uint32_t CSR_no, uint32_t value);

uint32_t io_read(uint32_t addr, uint32_t* data);
uint32_t io_write(uint32_t addr, uint32_t* data);
