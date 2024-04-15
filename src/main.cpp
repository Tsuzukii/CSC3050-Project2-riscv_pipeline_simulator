#include <cstdio>
#include <cstdlib>

#include <elfio/elfio.hpp>

#include "simulator/memory_manager.h"
#include "simulator/simulator.h"

bool parse_params(int argc, char** argv);
void load_elf_to_memory(ELFIO::elfio* reader, MemoryManager* memory);

char* elf_file = nullptr;
uint32_t stack_base_addr = 0x80000000;
uint32_t stack_size = 0x400000;
MemoryManager memory;
Simulator simulator(&memory);

int main(int argc, char** argv) {
    // not necessary to implement, can delete
    if (!parse_params(argc, argv)) {
        exit(-1);
    }

    bool arg_count = argc == 3;
    if (arg_count) {
        std::string history_arg = argv[2];
        bool history_flag = history_arg == "-history";
        if (history_flag){
            simulator.his_print = true;
        }
    }

    // read elf file
    elf_file = argv[1];
    ELFIO::elfio reader;
    if (!reader.load(elf_file)) {
        fprintf(stderr, "Fail to load ELF file %s!\n", elf_file);
        return -1;
    }

    load_elf_to_memory(&reader, &memory);

    // get entry point
    simulator.pc = reader.get_entry();

    // init stack
    simulator.init_stack(stack_base_addr, stack_size);
    simulator.simulate();

    return 0;
}

// not necessary to implement, can delete
bool parse_params(int argc, char** argv) {
    // Read Parameters implementation
    return true;
}

// load elf file to memory
void load_elf_to_memory(ELFIO::elfio* reader, MemoryManager* memory) {
    ELFIO::Elf_Half seg_num = reader->segments.size();
    for (int i = 0; i < seg_num; ++i) {
        const ELFIO::segment* pseg = reader->segments[i];

        uint32_t memory_size = pseg->get_memory_size();
        uint32_t offset = pseg->get_virtual_address();

        // check if the address space is larger than 32bit
        if (offset + memory_size > 0xFFFFFFFF) {
            fprintf(
                    stderr,
                    "ELF address space larger than 32bit! Seg %d has max addr of 0x%x\n",
                    i, offset + memory_size);
            exit(-1);
        }

        uint32_t filesz = pseg->get_file_size();
        uint32_t memsz = pseg->get_memory_size();
        uint32_t addr = (uint32_t)pseg->get_virtual_address();

        for (uint32_t p = addr; p < addr + memsz; ++p) {
            if (!memory->is_page_exit(p)) {
                memory->add_page(p);
            }

            if (p < addr + filesz) {
                memory->set_byte(p, pseg->get_data()[p - addr]);
            } else {
                memory->set_byte(p, 0);
            }
        }
    }
}
