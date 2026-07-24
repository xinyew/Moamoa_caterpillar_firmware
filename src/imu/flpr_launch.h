#ifndef FLPR_LAUNCH_H
#define FLPR_LAUNCH_H

/**
 * @brief Copy the embedded FLPR firmware into coprocessor SRAM and
 * start the RISC-V core.  Call once, early in main().
 */
void flpr_launch(void);

#endif /* FLPR_LAUNCH_H */
