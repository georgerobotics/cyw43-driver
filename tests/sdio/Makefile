CYW43_TOP ?= ../..

RM ?= rm
MKDIR ?= mkdir

BUILD ?= build

CC ?= cc

CFLAGS += -I.
CFLAGS += -I$(CYW43_TOP)
CFLAGS += -std=c99
CFLAGS += -Wall -Wpedantic -Werror
CFLAGS += -Wno-unused-local-typedefs
CFLAGS += -m32

SRC += \
	main.c \
	mock_sdio.c \
	src/cyw43_ctrl.c \
	src/cyw43_ll.c \
	src/cyw43_lwip.c \
	src/cyw43_sdio.c \
	src/cyw43_stats.c \

OBJ += $(addprefix $(BUILD)/,$(SRC:.c=.o))

.PHONY: all
all: $(BUILD)/test

.PHONY: clean
clean:
	$(RM) -rf $(BUILD)

.PHONY: test
test: $(BUILD)/test
	./$(BUILD)/test | diff - test.exp

$(BUILD)/test: $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^

vpath %.c . $(CYW43_TOP)
$(BUILD)/%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

OBJ_DIRS = $(sort $(dir $(OBJ)))
$(OBJ): | $(OBJ_DIRS)
$(OBJ_DIRS):
	$(MKDIR) -p $@

# "Include-what-you-use"-test compilation for public headers
PUBLIC_HEADER := \
	src/cyw43.h \
	src/cyw43_btbus.h \
	src/cyw43_country.h \
	src/cyw43_sdio.h \
	src/cyw43_spi.h \
	src/cyw43_stats.h \

vpath %.h . $(CYW43_TOP)

PUBLIC_HEADER_GCH := $(patsubst %,$(BUILD)/%.gch,$(PUBLIC_HEADER))

# Static pattern rule testing header compilation by making a pre-compiled header of each
$(PUBLIC_HEADER_GCH): $(BUILD)/%.h.gch: %.h
# Pass -Wno-pedantic to suppress "empty translation unit" errors
	$(CC) $(CFLAGS) -Wno-pedantic -c $< -o $@

# Add to phony test target trigger
test: $(PUBLIC_HEADER_GCH)

# Dependency generation
%.o: %.d
%.gch: %.d
CFLAGS += -MP -MMD
DEP := $(OBJ:.o=.d) $(PUBLIC_HEADER_GCH:.gch=.d)
$(DEP):

-include $(DEP)
