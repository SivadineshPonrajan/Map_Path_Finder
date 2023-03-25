

LDFLAGS            :=
CC                 := g++ -g -O0
OBJDIR             := ./build

# CFLAGS_COMMON      := -Wall -Wshadow -Wextra -Wno-unused-parameter -Werror
CFLAGS_COMMON      += -std=c++14

CFLAGS_DEBUG       := $(CFLAGS_COMMON) -O -g
CFLAGS             := $(CFLAGS_COMMON) -O3


TARGET             := graph_traversal

SRC_FILES          := \
  ./graph_traversal.cpp\
  ./graph.cpp


OBJECT_FILES       := $(addprefix  $(OBJDIR)/,$(notdir $(SRC_FILES:.cpp=.r.o)))
OBJECT_FILES_DEBUG := $(addprefix  $(OBJDIR)/,$(notdir $(SRC_FILES:.cpp=.d.o)))
DEP_FILES          := $(addprefix  $(OBJDIR)/,$(notdir $(SRC_FILES:.h=.r.P)))
DEP_FILES_DEBUG    := $(addprefix  $(OBJDIR)/,$(notdir $(SRC_FILES:.cpp=.d.P)))


all: $(TARGET)


$(TARGET): $(OBJECT_FILES) Makefile
	$(CC) -o $@ $(OBJECT_FILES) $(LDFLAGS)

debug: $(OBJECT_FILES_DEBUG) Makefile
	$(CC) -o $(TARGET).$@ $(OBJECT_FILES_DEBUG) $(LDFLAGS)

$(OBJECT_FILES)        :| $(OBJDIR)
$(OBJECT_FILES_DEBUG)  :| $(OBJDIR)


$(OBJDIR)/%.r.o : ./%.cpp
	@printf "Compiling release version of $<\n"
	$(CC) -c -MMD -MF $(OBJDIR)/$*.r.d -MT $(OBJDIR)/$*.r.o $(CFLAGS) -o $@ $<
	@cp $(OBJDIR)/$*.r.d $(OBJDIR)/$*.r.P
	@sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $(OBJDIR)/$*.r.d >> $(OBJDIR)/$*.r.P
	@rm -f $(OBJDIR)/$*.r.d

$(OBJDIR)/%.d.o : ./%.cpp
	@printf "Compiling debug version of $<\n"
	$(CC) -c -MMD -MF $(OBJDIR)/$*.d.d -MT $(OBJDIR)/$*.d.o $(CFLAGS_DEBUG) -o $@ $<
	@cp $(OBJDIR)/$*.d.d $(OBJDIR)/$*.d.P
	@sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $(OBJDIR)/$*.d.d >> $(OBJDIR)/$*.d.P
	@rm -f $(OBJDIR)/$*.d.d


$(OBJDIR):
	mkdir -p $@


clean:
	$(RM) $(TARGET) $(TARGET).debug $(OBJECT_FILES)  $(OBJECT_FILES_DEBUG) $(DEP_FILES) $(DEP_FILES_DEBUG)


.SUFFIXES:              # Delete the default suffixes
.SUFFIXES: .cpp .o .h   # Define "CPP" language suffix list

-include $(DEP_FILES)
-include $(DEP_FILES_DEBUG)


# IDIR =../include
# CC=g++
# CFLAGS=-I$(IDIR)

# ODIR=obj
# # LDIR =../lib
# LDIR =./

# LIBS=-lm

# _DEPS = graph.h
# DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

# _OBJ =  main.o graph.o
# OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))


# $(ODIR)/%.o: %.cpp $(DEPS)
# 	$(CC) -c -o $@ $< $(CFLAGS)

# main: $(OBJ)
# 	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

# .PHONY: clean

# clean:
# 	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 

# CC=/usr/bin/clang++
# # # CFLAGS=--I. 
# CFLAGS=-fcolor-diagnostics -std=c++11 -fansi-escape-codes -I.
# DEPS = graph.h
# #
# %.o: %.cpp $(DEPS)
# 	g++ -fcolor-diagnostics -std=c++11 -fansi-escape-codes -g main.cpp -o main.o -g graph.cpp -o graph.o

# # main: main.o graph.o 
# # 	$(CC) -o main main.o graph.o 






# main: main.o graph.o
# 	g++ -std=c++11 -fansi-escape-codes -o main main.o graph.o