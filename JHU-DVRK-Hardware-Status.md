<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](http://doctoc.herokuapp.com/)*

- [Current Setup](#current-setup)
- [Manufacture Batch](#manufacture-batch)
- [FPGA boards](#fpga-boards)
- [QLA boards](#qla-boards)
- [Log](#log)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

This page documents all FPGA/QLA boards and DVRK setup and status in JHU. 

Maintainer: Zihan Chen   
Email: zihan dot chen at jhu dot edu

### Current Setup 
| Robot    |   Axis    |    S/N    |   FPGA    |   QLA   |  Status  | 
| -------- | --------- | --------- | --------- | ------- | -------- |
| MTML     |  1-4 |  M28003   | 3792-21   | 3791-18 | GOOD |
| MTML     |  5-8 |  M28003   | 3792-41   | 3791-31 | GOOD |
| MTMR     |  1-4 |  M28247   | 3792-22   | 3791-14 | GOOD |
| MTMR     |  5-8 |  M28247   | 3792-43   | 3791-41 | GOOD |
| PSM1     |  1-4 |  M28007   | 3116-01   | 3791-15 | GOOD |
| PSM1     |  5-8 |  M28007   | 3116-03   | 3791-19 | GOOD |
| PSM2     |  1-4 |  M27374   | 3792-40   | 3791-30 | GOOD |
| PSM2     |  5-8 |  M27374   | 3792-12   | 3791-10 | GOOD |

### Manufacture Batch 

| Batch    |  FPGA S/N |    Number    | QLA S/N |  Number  | 
| -------- | --------- | ------------ | ------- | -------- | 
| Pilot    |  3116-XX  |  05/05       | 3174-XX |   05/05  |
| Batch 1  |  3792-XX  |  10/44       | 3791-XX |   10/44  |
| Batch 2  |  3985-XX  |  01/68       | 3984-XX |   01/68  |


### FPGA boards 

**Pilot**  

| S/N     |   Status   |   Location    |   Comment                              |
| ------- | ---------- | ------------- | -------------------------------------- |
| 3116-01 |  GOOD      |   JHU         |  FireWire Port Issue (Fixed)           | 
| 3116-02 |  GOOD      |   JHU         |  Ethernet                              | 
| 3116-03 |  GOOD      |   JHU         |                                        | 
| 3116-04 |  FAIL      |   JHU         |  Power Issue                           | 
| 3116-05 |  GOOD      |   JHU         |  SUJ Development Anton                 | 


**Batch 1**

| S/N     |   Status   |   Location    |   Comment                              |
| ------- | ---------- | ------------- | -------------------------------------- |
| 3792-21 |  GOOD      |   JHU         |  DVRK BOX ???                          | 
| 3792-41 |  GOOD      |   JHU         |  DVRK BOX ???                          | 
| 3792-22 |  GOOD      |   JHU         |  DVRK BOX ???                          | 
| 3792-43 |  GOOD      |   JHU         |  DVRK BOX ???                          | 
| 3792-12 |  GOOD      |   JHU         |                                        | 
| 3792-37 |  GOOD      |   JHU         |  Power issue when mated with QLA       | 
| 3792-40 |  GOOD      |   JHU         |                                        | 
| 3792-27 |  GOOD      |   JHU         |                                        | 
| 3792-42 |  GOOD      |   AVCOM       |  Test firmware for AVCOM               | 
| 3792-39 |  GOOD      |   JHU         |  Ethernet                              | 

**Batch 2**

| S/N     |   Status   |   Location    |   Comment                              |
| ------- | ---------- | ------------- | -------------------------------------- |
| 3985-66 |  GOOD      |   JHU         |                                        | 


**Batch 3**   

| S/N     | Status | Location | Comment     |
|---------|--------|----------|-------------|
| 4266-17 | BAD    | JHU      | Power Issue |

### QLA boards 

**Pilot**  

| S/N     |   Status   |   Location    |   Comment                              |
| ------- | ---------- | ------------- | -------------------------------------- |
| 3174-01 |  GOOD      |   JHU         |                                        | 
| 3174-02 |  GOOD      |   JHU         |                                        | 
| 3174-03 |  GOOD      |   JHU         |                                        | 
| 3174-04 |  FAIL      |   JHU         |  Axis 4 no power feedback (bool)       | 
| 3174-05 |  GOOD      |   JHU         |  SUJ Development Anton                 | 

**Batch 1**

| S/N     |   Status   |   Location    |   Comment                              |
| ------- | ---------- | ------------- | -------------------------------------- |
| 3791-18 |  GOOD      |   JHU         | Axis3 VREF = 0, Now uses Axis4 VREF    | 
| 3791-07 |  GOOD      |   JHU         | Axis 3 Cur Issue Fixed ZC R52          | 
| 3791-08 |  GOOD      |   JHU         |                                        | 
| 3791-31 |  GOOD      |   JHU         |                                        | 
| 3791-14 |  GOOD      |   JHU         |                                        | 
| 3791-41 |  GOOD      |   JHU         |                                        | 
| 3791-15 |  GOOD      |   JHU         |                                        | 
| 3791-19 |  GOOD      |   JHU         |                                        | 
| 3791-30 |  GOOD      |   JHU         |                                        | 
| 3791-10 |  GOOD      |   JHU         |                                        | 

**Batch 2**

| S/N     |   Status   |   Location    |   Comment                              |
| ------- | ---------- | ------------- | -------------------------------------- |
| 3984-13 |  GOOD      |   JHU         |                                        | 

**Batch 3**

| S/N     | Status | Location | Comment     |
|---------|--------|----------|-------------|
| 4265-14 | GOOD   | JHU      | Short under J3 (Enc/Din) Fixed    |

### Log 

