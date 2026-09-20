# carefully! not compile *.c files!!
# Pre-compiler and Compiler flags
CXX_FLAGS :=  -std=c++14 -pthread -march=armv8-a -mcpu=cortex-a53 -O3
PRE_FLAGS := 

# Project directory structure
BIN := ../
SRC := .
LIB := 
INC := ../include /usr/include /usr/include/vio/
MAINFILE := $(SRC)/TheOne.cpp

# Build directories and output
TARGET := $(BIN)/the1
BUILD := ../buildcpp
# Library search directories and flags
EXT_LIB :=/usr/lib /usr/lib/hbbpu/

OPENCV_LIBS := $(shell pkg-config --libs opencv4)
OPENCV_FLAGS := $(shell pkg-config --cflags opencv4)
BLUETOOTH_LIBS := -lbluetooth
LDFLAGS :=-ldnn -lpthread -lspcdev $(OPENCV_LIBS) -lX11 $(BLUETOOTH_LIBS)
LDPATHS := $(addprefix -L,$(LIB) $(EXT_LIB))

# Include directories
INC_DIRS := $(INC) $(shell find $(SRC) -type d) 
OPENCV_INCLUDES := $(shell pkg-config --cflags opencv4)
INC_FLAGS := $(addprefix -I,$(INC_DIRS)) $(OPENCV_INCLUDES)

# Construct build output and dependency filenames
SRCS := $(shell find $(SRC) -name "*.cpp")
OBJS := $(subst $(SRC)/,$(BUILD)/,$(addsuffix .o,$(basename $(SRCS))))
DEPS := $(OBJS:.o=.d)


# Dependency generation flags
DEPFLAGS = -MT $@ -MMD -MP -MF $(BUILD)/$*.d

# Build task
build: all

# Main task
all: $(TARGET)

# Task producing target from built files
$(TARGET): $(OBJS)
	@echo "Building..."
	$(CXX) $(CXX_FLAGS) $(OBJS) -o $@ $(LDPATHS) $(LDFLAGS)

# Compile all cpp files
$(BUILD)/%.o: $(SRC)/%.cpp $(BUILD)/%.d
	mkdir -p $(dir $@)
	$(CXX) $(CXX_FLAGS) $(PRE_FLAGS) $(INC_FLAGS) $(DEPFLAGS) -c -o $@ $< `pkg-config --cflags --libs opencv4`

# Dependency file generation rule
$(BUILD)/%.d: $(SRC)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXX_FLAGS) $(PRE_FLAGS) $(INC_FLAGS) -MM -MT "$(BUILD)/$*.o $(BUILD)/$*.d" $< -o $@

# Clean task
.PHONY: clean
clean:
	@echo "Clearing..."
	rm -rf $(BUILD)

# Include all dependencies
-include $(DEPS)
