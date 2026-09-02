# SigmaStudio+ Project

This directory contains the SigmaStudio+ project and generated files for the SigmaDSP ADAU1466.

## Updating the DSP Project

After modifying [`JUMBLEQ_DSP.ssprj`](./JUMBLEQ_DSP.ssprj), perform the following operations in SigmaStudio+:

1. Run **Link Compile**.
2. Run **Export System Files**.

The exported files are written to [`exported_code`](./exported_code/).

## Integrating Changes into JUMBLEQ

To include the updated DSP project in JUMBLEQ, copy the following generated header files from [`exported_code`](./exported_code/) to [`STM32CubeIDE/JUMBLEQ/Appli/Core/Inc`](../STM32CubeIDE/JUMBLEQ/Appli/Core/Inc/), replacing the existing files:

- `JUMBLEQ_DSP_ADAU146xSchematic_1.h`
- `JUMBLEQ_DSP_ADAU146xSchematic_1_Defines.h`
- `JUMBLEQ_DSP_ADAU146xSchematic_1_PARAM.h`

<img width="480" height="789" alt="ss_input1" src="https://github.com/user-attachments/assets/ecda6ea1-8714-4d81-a17d-f688ab1e55fb" />
<img width="959" height="618" alt="ss_input2" src="https://github.com/user-attachments/assets/237f5551-20a5-4e13-bc06-bc7fecabb4a5" />
<img width="384" height="594" alt="ss_sw1" src="https://github.com/user-attachments/assets/5ac31b1f-c508-41df-959b-caed95e8570f" />
<img width="310" height="320" alt="ss_sw2" src="https://github.com/user-attachments/assets/91ba9ce9-0419-44fe-abf7-2cf17e72f39d" />
<img width="405" height="558" alt="ss_fader" src="https://github.com/user-attachments/assets/6d36b6ba-e39f-420b-9d99-4695698396c4" />
<img width="406" height="403" alt="ss_read" src="https://github.com/user-attachments/assets/dcccf577-2000-42cd-9e0f-b0cc66a42b91" />
<img width="961" height="795" alt="ss_output1" src="https://github.com/user-attachments/assets/9a640d63-c274-4d97-8f79-9379eed70a35" />
<img width="604" height="333" alt="ss_output2" src="https://github.com/user-attachments/assets/6afdac35-965a-4c8f-8f71-380e987fe199" />
<img width="440" height="567" alt="ss_output3" src="https://github.com/user-attachments/assets/df4c5fbd-f2ae-49bd-9240-1a73c689411a" />
