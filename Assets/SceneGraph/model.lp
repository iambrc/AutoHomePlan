\ LP format - for model browsing. Use MPS format to capture full model detail.
\ Signature: 0xd0bf9568cccc7000
Minimize
  - 0.4 x_0 - 0.6 l_0 - 0.4 w_0 - 0.2 h_0 - 0.2 l_1 - 0.2 w_1 - 0.16 h_1
   - 0.2 l_2 - 0.2 w_2 - 0.16 h_2 + 0.4 x_3 - 0.4 y_3 - 0.6 l_3 - 0.1 w_3
   - 0.4 h_3 + 0.22 x_4 + 0.4 y_4 - 0.29 l_4 - 0.3 w_4 - 0.24 h_4
   - 0.22 x_5 - 0.15 l_5 - 0.16 w_5 - 0.16 h_5 - x_6 - y_6 - 0.08 l_6
   - 0.08 w_6 - 0.5 z_6 - 0.2 h_6 - 0.04 l_7 - 0.04 w_7 - 0.04 h_7
   + 2.231300000876189 Constant + [ 2 x_0 ^2 - 2 x_0 * l_0 - 4 x_0 * x_3
   - 2 x_0 * l_3 + 4 y_0 ^2 - 4 y_0 * y_2 - 2 y_0 * w_2 - 4 y_0 * y_1
   + 2 y_0 * w_1 - 2 l_0 * w_0 + 2.5 l_0 ^2 + 2 l_0 * x_3 + l_0 * l_3
   + 3 w_0 ^2 + 2 w_0 * y_2 + w_0 * w_2 - 2 w_0 * y_1 + w_0 * w_1
   + 2 h_0 ^2 + 2 y_1 ^2 - 2 y_1 * w_1 - 2 l_1 * w_1 + 2 l_1 ^2
   + 2.5 w_1 ^2 + 2 h_1 ^2 + 2 y_2 ^2 + 2 y_2 * w_2 - 2 l_2 * w_2
   + 2 l_2 ^2 + 2.5 w_2 ^2 + 2 h_2 ^2 + 2 x_3 ^2 + 2 x_3 * l_3 + 2 y_3 ^2
   - 2 y_3 * w_3 - 4 y_3 * y_4 - 2 y_3 * w_4 - 2 l_3 * w_3 + 2.5 l_3 ^2
   + 2.5 w_3 ^2 + 2 w_3 * y_4 + w_3 * w_4 + 2 h_3 ^2 - 8 x_4 * x_5
   + 2 x_4 * l_5 + 6 x_4 ^2 + 2 x_4 * l_4 - 4 x_4 * x_7 + 6 y_4 ^2
   + 2 y_4 * w_4 - 4 y_4 * y_5 - 4 y_4 * y_7 - 2 l_4 * w_4 + 2.5 l_4 ^2
   - 2 l_4 * x_5 + l_4 * l_5 + 2.5 w_4 ^2 + 2 h_4 ^2 + 4 x_5 ^2
   - 2 x_5 * l_5 + 2 y_5 ^2 - 2 l_5 * w_5 + 2.5 l_5 ^2 + 2 w_5 ^2
   + 2 h_5 ^2 + 2 x_6 ^2 + 2 y_6 ^2 - 2 l_6 * w_6 + 2 l_6 ^2 + 2 w_6 ^2
   + 2 z_6 ^2 + 2 h_6 ^2 + 2 x_7 ^2 + 2 y_7 ^2 - 2 l_7 * w_7 + 2 l_7 ^2
   + 2 w_7 ^2 + 2 h_7 ^2 ] / 2 
Subject To
 Inside_Object_0_x_left: x_0 - 0.5 l_0 >= 0.00999999977648258
 Inside_Object_0_x_right: x_0 + 0.5 l_0 <= 0.9900000002235174
 Inside_Object_0_y_back: y_0 - 0.5 w_0 >= 0.00999999977648258
 Inside_Object_0_y_front: y_0 + 0.5 w_0 <= 0.9900000002235174
 Inside_Object_0_z_bottom: z_0 - 0.5 h_0 >= 0
 Inside_Object_0_z_top: z_0 + 0.5 h_0 <= 0.3
 Size_Tolerance_Object_0_l_min: l_0 >= 0.3
 Size_Tolerance_Object_0_l_max: l_0 <= 0.5
 Size_Tolerance_Object_0_w_min: w_0 >= 0.1
 Size_Tolerance_Object_0_w_max: w_0 <= 0.3
 Size_Tolerance_Object_0_h_min: h_0 >= 0.1
 Size_Tolerance_Object_0_h_max: h_0 <= 0.1
 Inside_Object_1_x_left: x_1 - 0.5 l_1 >= 0.00999999977648258
 Inside_Object_1_x_right: x_1 + 0.5 l_1 <= 0.9900000002235174
 Inside_Object_1_y_back: y_1 - 0.5 w_1 >= 0.00999999977648258
 Inside_Object_1_y_front: y_1 + 0.5 w_1 <= 0.9900000002235174
 Inside_Object_1_z_bottom: z_1 - 0.5 h_1 >= 0
 Inside_Object_1_z_top: z_1 + 0.5 h_1 <= 0.3
 Size_Tolerance_Object_1_l_min: l_1 >= 0.08
 Size_Tolerance_Object_1_l_max: l_1 <= 0.12
 Size_Tolerance_Object_1_w_min: w_1 >= 0.08
 Size_Tolerance_Object_1_w_max: w_1 <= 0.12
 Size_Tolerance_Object_1_h_min: h_1 >= 0.08
 Size_Tolerance_Object_1_h_max: h_1 <= 0.08
 Inside_Object_2_x_left: x_2 - 0.5 l_2 >= 0.00999999977648258
 Inside_Object_2_x_right: x_2 + 0.5 l_2 <= 0.9900000002235174
 Inside_Object_2_y_back: y_2 - 0.5 w_2 >= 0.00999999977648258
 Inside_Object_2_y_front: y_2 + 0.5 w_2 <= 0.9900000002235174
 Inside_Object_2_z_bottom: z_2 - 0.5 h_2 >= 0
 Inside_Object_2_z_top: z_2 + 0.5 h_2 <= 0.3
 Size_Tolerance_Object_2_l_min: l_2 >= 0.08
 Size_Tolerance_Object_2_l_max: l_2 <= 0.12
 Size_Tolerance_Object_2_w_min: w_2 >= 0.08
 Size_Tolerance_Object_2_w_max: w_2 <= 0.12
 Size_Tolerance_Object_2_h_min: h_2 >= 0.08
 Size_Tolerance_Object_2_h_max: h_2 <= 0.08
 Inside_Object_3_x_left: x_3 - 0.5 l_3 >= 0.00999999977648258
 Inside_Object_3_x_right: x_3 + 0.5 l_3 <= 0.9900000002235174
 Inside_Object_3_y_back: y_3 - 0.5 w_3 >= 0.00999999977648258
 Inside_Object_3_y_front: y_3 + 0.5 w_3 <= 0.9900000002235174
 Inside_Object_3_z_bottom: z_3 - 0.5 h_3 >= 0
 Inside_Object_3_z_top: z_3 + 0.5 h_3 <= 0.3
 Size_Tolerance_Object_3_l_min: l_3 >= 0.35
 Size_Tolerance_Object_3_l_max: l_3 <= 0.45
 Size_Tolerance_Object_3_w_min: w_3 >= 0.12
 Size_Tolerance_Object_3_w_max: w_3 <= 0.18
 Size_Tolerance_Object_3_h_min: h_3 >= 0.2
 Size_Tolerance_Object_3_h_max: h_3 <= 0.2
 Inside_Object_4_x_left: x_4 - 0.5 l_4 >= 0.00999999977648258
 Inside_Object_4_x_right: x_4 + 0.5 l_4 <= 0.9900000002235174
 Inside_Object_4_y_back: y_4 - 0.5 w_4 >= 0.00999999977648258
 Inside_Object_4_y_front: y_4 + 0.5 w_4 <= 0.9900000002235174
 Inside_Object_4_z_bottom: z_4 - 0.5 h_4 >= 0
 Inside_Object_4_z_top: z_4 + 0.5 h_4 <= 0.3
 Size_Tolerance_Object_4_l_min: l_4 >= 0.13
 Size_Tolerance_Object_4_l_max: l_4 <= 0.17
 Size_Tolerance_Object_4_w_min: w_4 >= 0.23
 Size_Tolerance_Object_4_w_max: w_4 <= 0.27
 Size_Tolerance_Object_4_h_min: h_4 >= 0.12
 Size_Tolerance_Object_4_h_max: h_4 <= 0.12
 Inside_Object_5_x_left: x_5 - 0.5 l_5 >= 0.00999999977648258
 Inside_Object_5_x_right: x_5 + 0.5 l_5 <= 0.9900000002235174
 Inside_Object_5_y_back: y_5 - 0.5 w_5 >= 0.00999999977648258
 Inside_Object_5_y_front: y_5 + 0.5 w_5 <= 0.9900000002235174
 Inside_Object_5_z_bottom: z_5 - 0.5 h_5 >= 0
 Inside_Object_5_z_top: z_5 + 0.5 h_5 <= 0.3
 Size_Tolerance_Object_5_l_min: l_5 >= 0.06
 Size_Tolerance_Object_5_l_max: l_5 <= 0.1
 Size_Tolerance_Object_5_w_min: w_5 >= 0.06
 Size_Tolerance_Object_5_w_max: w_5 <= 0.1
 Size_Tolerance_Object_5_h_min: h_5 >= 0.08
 Size_Tolerance_Object_5_h_max: h_5 <= 0.08
 Inside_Object_6_x_left: x_6 - 0.5 l_6 >= 0.00999999977648258
 Inside_Object_6_x_right: x_6 + 0.5 l_6 <= 0.9900000002235174
 Inside_Object_6_y_back: y_6 - 0.5 w_6 >= 0.00999999977648258
 Inside_Object_6_y_front: y_6 + 0.5 w_6 <= 0.9900000002235174
 Inside_Object_6_z_bottom: z_6 - 0.5 h_6 >= 0
 Inside_Object_6_z_top: z_6 + 0.5 h_6 <= 0.3
 Size_Tolerance_Object_6_l_min: l_6 >= 0.04
 Size_Tolerance_Object_6_l_max: l_6 <= 0.04
 Size_Tolerance_Object_6_w_min: w_6 >= 0.04
 Size_Tolerance_Object_6_w_max: w_6 <= 0.04
 Size_Tolerance_Object_6_h_min: h_6 >= 0.1
 Size_Tolerance_Object_6_h_max: h_6 <= 0.1
 Inside_Object_7_x_left: x_7 - 0.5 l_7 >= 0.00999999977648258
 Inside_Object_7_x_right: x_7 + 0.5 l_7 <= 0.9900000002235174
 Inside_Object_7_y_back: y_7 - 0.5 w_7 >= 0.00999999977648258
 Inside_Object_7_y_front: y_7 + 0.5 w_7 <= 0.9900000002235174
 Inside_Object_7_z_bottom: z_7 - 0.5 h_7 >= 0
 Inside_Object_7_z_top: z_7 + 0.5 h_7 <= 0.3
 Size_Tolerance_Object_7_l_min: l_7 >= 0.02
 Size_Tolerance_Object_7_l_max: l_7 <= 0.02
 Size_Tolerance_Object_7_w_min: w_7 >= 0.02
 Size_Tolerance_Object_7_w_max: w_7 <= 0.02
 Size_Tolerance_Object_7_h_min: h_7 >= 0.02
 Size_Tolerance_Object_7_h_max: h_7 <= 0.02
 On_Floor_Object_0: z_0 - 0.5 h_0 = 0
 On_Floor_Object_1: z_1 - 0.5 h_1 = 0
 On_Floor_Object_2: z_2 - 0.5 h_2 = 0
 On_Floor_Object_3: z_3 - 0.5 h_3 = 0
 On_Floor_Object_4: z_4 - 0.5 h_4 = 0
 On_Floor_Object_5: z_5 - 0.5 h_5 = 0
 Hanging_Object_6: z_6 + 0.5 h_6 = 0.3
 Object_0_FrontOf_Object_2: y_0 - 0.5 w_0 - y_2 - 0.5 w_2 >= 0
 Object_1_AlignWith_Object_0: - x_0 - 0.5 l_0 + x_1 + 0.5 l_1 = 0
 Object_1_FrontOf_Object_0: - y_0 - 0.5 w_0 + y_1 - 0.5 w_1 >= 0
 Object_2_AlignWith_Object_0: - x_0 - 0.5 l_0 + x_2 + 0.5 l_2 = 0
 Object_3_LeftOf_Object_0: - x_0 + 0.5 l_0 + x_3 + 0.5 l_3 <= 0
 Object_3_FrontOf_Object_4: y_3 - 0.5 w_3 - y_4 - 0.5 w_4 >= 0
 Object_4_LeftOf_Object_5: x_4 + 0.5 l_4 - x_5 + 0.5 l_5 <= 0
 Object_5_CloseBy_Object_4ieqa: - x_4 - 0.5 l_4 + x_5 - 0.5 l_5
   + 2.260000000894069 C397 <= 2.260000000894069
 Object_5_CloseBy_Object_4ieqb: - x_4 + 0.5 l_4 + x_5 + 0.5 l_5
   - 2.260000000894069 C396 >= -2.260000000894069
 Object_5_CloseBy_Object_4ieqc: - y_4 - 0.5 w_4 + y_5 - 0.5 w_5
   + 2.260000000894069 C398 <= 2.260000000894069
 Object_5_CloseBy_Object_4ieqd: - y_4 + 0.5 w_4 + y_5 + 0.5 w_5
   - 2.260000000894069 C399 >= -2.260000000894069
 Object_5_CloseBy_Object_4ieqe: C396 + C397 + C398 + C399 <= 1
 Object_7_Above_Object_4: - z_4 - 0.5 h_4 + z_7 - 0.5 h_7 = 0
 NonOverlap_Object_0and_Object_4R: x_0 - 0.5 l_0 - x_4 - 0.5 l_4
   - 2.260000000894069 C400 >= -2.260000000894069
 NonOverlap_Object_0and_Object_4L: x_0 + 0.5 l_0 - x_4 + 0.5 l_4
   + 2.260000000894069 C401 <= 2.260000000894069
 NonOverlap_Object_0and_Object_4F: y_0 - 0.5 w_0 - y_4 - 0.5 w_4
   - 2.260000000894069 C402 >= -2.260000000894069
 NonOverlap_Object_0and_Object_4B: y_0 + 0.5 w_0 - y_4 + 0.5 w_4
   + 2.260000000894069 C403 <= 2.260000000894069
 NonOverlap_Object_0and_Object_4U: z_0 - 0.5 h_0 - z_4 - 0.5 h_4
   - 2.260000000894069 C404 >= -2.260000000894069
 NonOverlap_Object_0and_Object_4D: z_0 + 0.5 h_0 - z_4 + 0.5 h_4
   + 2.260000000894069 C405 <= 2.260000000894069
 NonOverlap_Object_0and_Object_4: C400 + C401 + C402 + C403 + C404 + C405
   >= 1
 NonOverlap_Object_0and_Object_5R: x_0 - 0.5 l_0 - x_5 - 0.5 l_5
   - 2.260000000894069 C406 >= -2.260000000894069
 NonOverlap_Object_0and_Object_5L: x_0 + 0.5 l_0 - x_5 + 0.5 l_5
   + 2.260000000894069 C407 <= 2.260000000894069
 NonOverlap_Object_0and_Object_5F: y_0 - 0.5 w_0 - y_5 - 0.5 w_5
   - 2.260000000894069 C408 >= -2.260000000894069
 NonOverlap_Object_0and_Object_5B: y_0 + 0.5 w_0 - y_5 + 0.5 w_5
   + 2.260000000894069 C409 <= 2.260000000894069
 NonOverlap_Object_0and_Object_5U: z_0 - 0.5 h_0 - z_5 - 0.5 h_5
   - 2.260000000894069 C410 >= -2.260000000894069
 NonOverlap_Object_0and_Object_5D: z_0 + 0.5 h_0 - z_5 + 0.5 h_5
   + 2.260000000894069 C411 <= 2.260000000894069
 NonOverlap_Object_0and_Object_5: C406 + C407 + C408 + C409 + C410 + C411
   >= 1
 NonOverlap_Object_0and_Object_6R: x_0 - 0.5 l_0 - x_6 - 0.5 l_6
   - 2.260000000894069 C412 >= -2.260000000894069
 NonOverlap_Object_0and_Object_6L: x_0 + 0.5 l_0 - x_6 + 0.5 l_6
   + 2.260000000894069 C413 <= 2.260000000894069
 NonOverlap_Object_0and_Object_6F: y_0 - 0.5 w_0 - y_6 - 0.5 w_6
   - 2.260000000894069 C414 >= -2.260000000894069
 NonOverlap_Object_0and_Object_6B: y_0 + 0.5 w_0 - y_6 + 0.5 w_6
   + 2.260000000894069 C415 <= 2.260000000894069
 NonOverlap_Object_0and_Object_6U: z_0 - 0.5 h_0 - z_6 - 0.5 h_6
   - 2.260000000894069 C416 >= -2.260000000894069
 NonOverlap_Object_0and_Object_6D: z_0 + 0.5 h_0 - z_6 + 0.5 h_6
   + 2.260000000894069 C417 <= 2.260000000894069
 NonOverlap_Object_0and_Object_6: C412 + C413 + C414 + C415 + C416 + C417
   >= 1
 NonOverlap_Object_0and_Object_7R: x_0 - 0.5 l_0 - x_7 - 0.5 l_7
   - 2.260000000894069 C418 >= -2.260000000894069
 NonOverlap_Object_0and_Object_7L: x_0 + 0.5 l_0 - x_7 + 0.5 l_7
   + 2.260000000894069 C419 <= 2.260000000894069
 NonOverlap_Object_0and_Object_7F: y_0 - 0.5 w_0 - y_7 - 0.5 w_7
   - 2.260000000894069 C420 >= -2.260000000894069
 NonOverlap_Object_0and_Object_7B: y_0 + 0.5 w_0 - y_7 + 0.5 w_7
   + 2.260000000894069 C421 <= 2.260000000894069
 NonOverlap_Object_0and_Object_7U: z_0 - 0.5 h_0 - z_7 - 0.5 h_7
   - 2.260000000894069 C422 >= -2.260000000894069
 NonOverlap_Object_0and_Object_7D: z_0 + 0.5 h_0 - z_7 + 0.5 h_7
   + 2.260000000894069 C423 <= 2.260000000894069
 NonOverlap_Object_0and_Object_7: C418 + C419 + C420 + C421 + C422 + C423
   >= 1
 NonOverlap_Object_1and_Object_3R: x_1 - 0.5 l_1 - x_3 - 0.5 l_3
   - 2.260000000894069 C424 >= -2.260000000894069
 NonOverlap_Object_1and_Object_3L: x_1 + 0.5 l_1 - x_3 + 0.5 l_3
   + 2.260000000894069 C425 <= 2.260000000894069
 NonOverlap_Object_1and_Object_3F: y_1 - 0.5 w_1 - y_3 - 0.5 w_3
   - 2.260000000894069 C426 >= -2.260000000894069
 NonOverlap_Object_1and_Object_3B: y_1 + 0.5 w_1 - y_3 + 0.5 w_3
   + 2.260000000894069 C427 <= 2.260000000894069
 NonOverlap_Object_1and_Object_3U: z_1 - 0.5 h_1 - z_3 - 0.5 h_3
   - 2.260000000894069 C428 >= -2.260000000894069
 NonOverlap_Object_1and_Object_3D: z_1 + 0.5 h_1 - z_3 + 0.5 h_3
   + 2.260000000894069 C429 <= 2.260000000894069
 NonOverlap_Object_1and_Object_3: C424 + C425 + C426 + C427 + C428 + C429
   >= 1
 NonOverlap_Object_1and_Object_4R: x_1 - 0.5 l_1 - x_4 - 0.5 l_4
   - 2.260000000894069 C430 >= -2.260000000894069
 NonOverlap_Object_1and_Object_4L: x_1 + 0.5 l_1 - x_4 + 0.5 l_4
   + 2.260000000894069 C431 <= 2.260000000894069
 NonOverlap_Object_1and_Object_4F: y_1 - 0.5 w_1 - y_4 - 0.5 w_4
   - 2.260000000894069 C432 >= -2.260000000894069
 NonOverlap_Object_1and_Object_4B: y_1 + 0.5 w_1 - y_4 + 0.5 w_4
   + 2.260000000894069 C433 <= 2.260000000894069
 NonOverlap_Object_1and_Object_4U: z_1 - 0.5 h_1 - z_4 - 0.5 h_4
   - 2.260000000894069 C434 >= -2.260000000894069
 NonOverlap_Object_1and_Object_4D: z_1 + 0.5 h_1 - z_4 + 0.5 h_4
   + 2.260000000894069 C435 <= 2.260000000894069
 NonOverlap_Object_1and_Object_4: C430 + C431 + C432 + C433 + C434 + C435
   >= 1
 NonOverlap_Object_1and_Object_5R: x_1 - 0.5 l_1 - x_5 - 0.5 l_5
   - 2.260000000894069 C436 >= -2.260000000894069
 NonOverlap_Object_1and_Object_5L: x_1 + 0.5 l_1 - x_5 + 0.5 l_5
   + 2.260000000894069 C437 <= 2.260000000894069
 NonOverlap_Object_1and_Object_5F: y_1 - 0.5 w_1 - y_5 - 0.5 w_5
   - 2.260000000894069 C438 >= -2.260000000894069
 NonOverlap_Object_1and_Object_5B: y_1 + 0.5 w_1 - y_5 + 0.5 w_5
   + 2.260000000894069 C439 <= 2.260000000894069
 NonOverlap_Object_1and_Object_5U: z_1 - 0.5 h_1 - z_5 - 0.5 h_5
   - 2.260000000894069 C440 >= -2.260000000894069
 NonOverlap_Object_1and_Object_5D: z_1 + 0.5 h_1 - z_5 + 0.5 h_5
   + 2.260000000894069 C441 <= 2.260000000894069
 NonOverlap_Object_1and_Object_5: C436 + C437 + C438 + C439 + C440 + C441
   >= 1
 NonOverlap_Object_1and_Object_6R: x_1 - 0.5 l_1 - x_6 - 0.5 l_6
   - 2.260000000894069 C442 >= -2.260000000894069
 NonOverlap_Object_1and_Object_6L: x_1 + 0.5 l_1 - x_6 + 0.5 l_6
   + 2.260000000894069 C443 <= 2.260000000894069
 NonOverlap_Object_1and_Object_6F: y_1 - 0.5 w_1 - y_6 - 0.5 w_6
   - 2.260000000894069 C444 >= -2.260000000894069
 NonOverlap_Object_1and_Object_6B: y_1 + 0.5 w_1 - y_6 + 0.5 w_6
   + 2.260000000894069 C445 <= 2.260000000894069
 NonOverlap_Object_1and_Object_6U: z_1 - 0.5 h_1 - z_6 - 0.5 h_6
   - 2.260000000894069 C446 >= -2.260000000894069
 NonOverlap_Object_1and_Object_6D: z_1 + 0.5 h_1 - z_6 + 0.5 h_6
   + 2.260000000894069 C447 <= 2.260000000894069
 NonOverlap_Object_1and_Object_6: C442 + C443 + C444 + C445 + C446 + C447
   >= 1
 NonOverlap_Object_1and_Object_7R: x_1 - 0.5 l_1 - x_7 - 0.5 l_7
   - 2.260000000894069 C448 >= -2.260000000894069
 NonOverlap_Object_1and_Object_7L: x_1 + 0.5 l_1 - x_7 + 0.5 l_7
   + 2.260000000894069 C449 <= 2.260000000894069
 NonOverlap_Object_1and_Object_7F: y_1 - 0.5 w_1 - y_7 - 0.5 w_7
   - 2.260000000894069 C450 >= -2.260000000894069
 NonOverlap_Object_1and_Object_7B: y_1 + 0.5 w_1 - y_7 + 0.5 w_7
   + 2.260000000894069 C451 <= 2.260000000894069
 NonOverlap_Object_1and_Object_7U: z_1 - 0.5 h_1 - z_7 - 0.5 h_7
   - 2.260000000894069 C452 >= -2.260000000894069
 NonOverlap_Object_1and_Object_7D: z_1 + 0.5 h_1 - z_7 + 0.5 h_7
   + 2.260000000894069 C453 <= 2.260000000894069
 NonOverlap_Object_1and_Object_7: C448 + C449 + C450 + C451 + C452 + C453
   >= 1
 NonOverlap_Object_2and_Object_3R: x_2 - 0.5 l_2 - x_3 - 0.5 l_3
   - 2.260000000894069 C454 >= -2.260000000894069
 NonOverlap_Object_2and_Object_3L: x_2 + 0.5 l_2 - x_3 + 0.5 l_3
   + 2.260000000894069 C455 <= 2.260000000894069
 NonOverlap_Object_2and_Object_3F: y_2 - 0.5 w_2 - y_3 - 0.5 w_3
   - 2.260000000894069 C456 >= -2.260000000894069
 NonOverlap_Object_2and_Object_3B: y_2 + 0.5 w_2 - y_3 + 0.5 w_3
   + 2.260000000894069 C457 <= 2.260000000894069
 NonOverlap_Object_2and_Object_3U: z_2 - 0.5 h_2 - z_3 - 0.5 h_3
   - 2.260000000894069 C458 >= -2.260000000894069
 NonOverlap_Object_2and_Object_3D: z_2 + 0.5 h_2 - z_3 + 0.5 h_3
   + 2.260000000894069 C459 <= 2.260000000894069
 NonOverlap_Object_2and_Object_3: C454 + C455 + C456 + C457 + C458 + C459
   >= 1
 NonOverlap_Object_2and_Object_4R: x_2 - 0.5 l_2 - x_4 - 0.5 l_4
   - 2.260000000894069 C460 >= -2.260000000894069
 NonOverlap_Object_2and_Object_4L: x_2 + 0.5 l_2 - x_4 + 0.5 l_4
   + 2.260000000894069 C461 <= 2.260000000894069
 NonOverlap_Object_2and_Object_4F: y_2 - 0.5 w_2 - y_4 - 0.5 w_4
   - 2.260000000894069 C462 >= -2.260000000894069
 NonOverlap_Object_2and_Object_4B: y_2 + 0.5 w_2 - y_4 + 0.5 w_4
   + 2.260000000894069 C463 <= 2.260000000894069
 NonOverlap_Object_2and_Object_4U: z_2 - 0.5 h_2 - z_4 - 0.5 h_4
   - 2.260000000894069 C464 >= -2.260000000894069
 NonOverlap_Object_2and_Object_4D: z_2 + 0.5 h_2 - z_4 + 0.5 h_4
   + 2.260000000894069 C465 <= 2.260000000894069
 NonOverlap_Object_2and_Object_4: C460 + C461 + C462 + C463 + C464 + C465
   >= 1
 NonOverlap_Object_2and_Object_5R: x_2 - 0.5 l_2 - x_5 - 0.5 l_5
   - 2.260000000894069 C466 >= -2.260000000894069
 NonOverlap_Object_2and_Object_5L: x_2 + 0.5 l_2 - x_5 + 0.5 l_5
   + 2.260000000894069 C467 <= 2.260000000894069
 NonOverlap_Object_2and_Object_5F: y_2 - 0.5 w_2 - y_5 - 0.5 w_5
   - 2.260000000894069 C468 >= -2.260000000894069
 NonOverlap_Object_2and_Object_5B: y_2 + 0.5 w_2 - y_5 + 0.5 w_5
   + 2.260000000894069 C469 <= 2.260000000894069
 NonOverlap_Object_2and_Object_5U: z_2 - 0.5 h_2 - z_5 - 0.5 h_5
   - 2.260000000894069 C470 >= -2.260000000894069
 NonOverlap_Object_2and_Object_5D: z_2 + 0.5 h_2 - z_5 + 0.5 h_5
   + 2.260000000894069 C471 <= 2.260000000894069
 NonOverlap_Object_2and_Object_5: C466 + C467 + C468 + C469 + C470 + C471
   >= 1
 NonOverlap_Object_2and_Object_6R: x_2 - 0.5 l_2 - x_6 - 0.5 l_6
   - 2.260000000894069 C472 >= -2.260000000894069
 NonOverlap_Object_2and_Object_6L: x_2 + 0.5 l_2 - x_6 + 0.5 l_6
   + 2.260000000894069 C473 <= 2.260000000894069
 NonOverlap_Object_2and_Object_6F: y_2 - 0.5 w_2 - y_6 - 0.5 w_6
   - 2.260000000894069 C474 >= -2.260000000894069
 NonOverlap_Object_2and_Object_6B: y_2 + 0.5 w_2 - y_6 + 0.5 w_6
   + 2.260000000894069 C475 <= 2.260000000894069
 NonOverlap_Object_2and_Object_6U: z_2 - 0.5 h_2 - z_6 - 0.5 h_6
   - 2.260000000894069 C476 >= -2.260000000894069
 NonOverlap_Object_2and_Object_6D: z_2 + 0.5 h_2 - z_6 + 0.5 h_6
   + 2.260000000894069 C477 <= 2.260000000894069
 NonOverlap_Object_2and_Object_6: C472 + C473 + C474 + C475 + C476 + C477
   >= 1
 NonOverlap_Object_2and_Object_7R: x_2 - 0.5 l_2 - x_7 - 0.5 l_7
   - 2.260000000894069 C478 >= -2.260000000894069
 NonOverlap_Object_2and_Object_7L: x_2 + 0.5 l_2 - x_7 + 0.5 l_7
   + 2.260000000894069 C479 <= 2.260000000894069
 NonOverlap_Object_2and_Object_7F: y_2 - 0.5 w_2 - y_7 - 0.5 w_7
   - 2.260000000894069 C480 >= -2.260000000894069
 NonOverlap_Object_2and_Object_7B: y_2 + 0.5 w_2 - y_7 + 0.5 w_7
   + 2.260000000894069 C481 <= 2.260000000894069
 NonOverlap_Object_2and_Object_7U: z_2 - 0.5 h_2 - z_7 - 0.5 h_7
   - 2.260000000894069 C482 >= -2.260000000894069
 NonOverlap_Object_2and_Object_7D: z_2 + 0.5 h_2 - z_7 + 0.5 h_7
   + 2.260000000894069 C483 <= 2.260000000894069
 NonOverlap_Object_2and_Object_7: C478 + C479 + C480 + C481 + C482 + C483
   >= 1
 NonOverlap_Object_3and_Object_5R: x_3 - 0.5 l_3 - x_5 - 0.5 l_5
   - 2.260000000894069 C484 >= -2.260000000894069
 NonOverlap_Object_3and_Object_5L: x_3 + 0.5 l_3 - x_5 + 0.5 l_5
   + 2.260000000894069 C485 <= 2.260000000894069
 NonOverlap_Object_3and_Object_5F: y_3 - 0.5 w_3 - y_5 - 0.5 w_5
   - 2.260000000894069 C486 >= -2.260000000894069
 NonOverlap_Object_3and_Object_5B: y_3 + 0.5 w_3 - y_5 + 0.5 w_5
   + 2.260000000894069 C487 <= 2.260000000894069
 NonOverlap_Object_3and_Object_5U: z_3 - 0.5 h_3 - z_5 - 0.5 h_5
   - 2.260000000894069 C488 >= -2.260000000894069
 NonOverlap_Object_3and_Object_5D: z_3 + 0.5 h_3 - z_5 + 0.5 h_5
   + 2.260000000894069 C489 <= 2.260000000894069
 NonOverlap_Object_3and_Object_5: C484 + C485 + C486 + C487 + C488 + C489
   >= 1
 NonOverlap_Object_3and_Object_6R: x_3 - 0.5 l_3 - x_6 - 0.5 l_6
   - 2.260000000894069 C490 >= -2.260000000894069
 NonOverlap_Object_3and_Object_6L: x_3 + 0.5 l_3 - x_6 + 0.5 l_6
   + 2.260000000894069 C491 <= 2.260000000894069
 NonOverlap_Object_3and_Object_6F: y_3 - 0.5 w_3 - y_6 - 0.5 w_6
   - 2.260000000894069 C492 >= -2.260000000894069
 NonOverlap_Object_3and_Object_6B: y_3 + 0.5 w_3 - y_6 + 0.5 w_6
   + 2.260000000894069 C493 <= 2.260000000894069
 NonOverlap_Object_3and_Object_6U: z_3 - 0.5 h_3 - z_6 - 0.5 h_6
   - 2.260000000894069 C494 >= -2.260000000894069
 NonOverlap_Object_3and_Object_6D: z_3 + 0.5 h_3 - z_6 + 0.5 h_6
   + 2.260000000894069 C495 <= 2.260000000894069
 NonOverlap_Object_3and_Object_6: C490 + C491 + C492 + C493 + C494 + C495
   >= 1
 NonOverlap_Object_3and_Object_7R: x_3 - 0.5 l_3 - x_7 - 0.5 l_7
   - 2.260000000894069 C496 >= -2.260000000894069
 NonOverlap_Object_3and_Object_7L: x_3 + 0.5 l_3 - x_7 + 0.5 l_7
   + 2.260000000894069 C497 <= 2.260000000894069
 NonOverlap_Object_3and_Object_7F: y_3 - 0.5 w_3 - y_7 - 0.5 w_7
   - 2.260000000894069 C498 >= -2.260000000894069
 NonOverlap_Object_3and_Object_7B: y_3 + 0.5 w_3 - y_7 + 0.5 w_7
   + 2.260000000894069 C499 <= 2.260000000894069
 NonOverlap_Object_3and_Object_7U: z_3 - 0.5 h_3 - z_7 - 0.5 h_7
   - 2.260000000894069 C500 >= -2.260000000894069
 NonOverlap_Object_3and_Object_7D: z_3 + 0.5 h_3 - z_7 + 0.5 h_7
   + 2.260000000894069 C501 <= 2.260000000894069
 NonOverlap_Object_3and_Object_7: C496 + C497 + C498 + C499 + C500 + C501
   >= 1
 NonOverlap_Object_4and_Object_6R: x_4 - 0.5 l_4 - x_6 - 0.5 l_6
   - 2.260000000894069 C502 >= -2.260000000894069
 NonOverlap_Object_4and_Object_6L: x_4 + 0.5 l_4 - x_6 + 0.5 l_6
   + 2.260000000894069 C503 <= 2.260000000894069
 NonOverlap_Object_4and_Object_6F: y_4 - 0.5 w_4 - y_6 - 0.5 w_6
   - 2.260000000894069 C504 >= -2.260000000894069
 NonOverlap_Object_4and_Object_6B: y_4 + 0.5 w_4 - y_6 + 0.5 w_6
   + 2.260000000894069 C505 <= 2.260000000894069
 NonOverlap_Object_4and_Object_6U: z_4 - 0.5 h_4 - z_6 - 0.5 h_6
   - 2.260000000894069 C506 >= -2.260000000894069
 NonOverlap_Object_4and_Object_6D: z_4 + 0.5 h_4 - z_6 + 0.5 h_6
   + 2.260000000894069 C507 <= 2.260000000894069
 NonOverlap_Object_4and_Object_6: C502 + C503 + C504 + C505 + C506 + C507
   >= 1
 NonOverlap_Object_5and_Object_6R: x_5 - 0.5 l_5 - x_6 - 0.5 l_6
   - 2.260000000894069 C508 >= -2.260000000894069
 NonOverlap_Object_5and_Object_6L: x_5 + 0.5 l_5 - x_6 + 0.5 l_6
   + 2.260000000894069 C509 <= 2.260000000894069
 NonOverlap_Object_5and_Object_6F: y_5 - 0.5 w_5 - y_6 - 0.5 w_6
   - 2.260000000894069 C510 >= -2.260000000894069
 NonOverlap_Object_5and_Object_6B: y_5 + 0.5 w_5 - y_6 + 0.5 w_6
   + 2.260000000894069 C511 <= 2.260000000894069
 NonOverlap_Object_5and_Object_6U: z_5 - 0.5 h_5 - z_6 - 0.5 h_6
   - 2.260000000894069 C512 >= -2.260000000894069
 NonOverlap_Object_5and_Object_6D: z_5 + 0.5 h_5 - z_6 + 0.5 h_6
   + 2.260000000894069 C513 <= 2.260000000894069
 NonOverlap_Object_5and_Object_6: C508 + C509 + C510 + C511 + C512 + C513
   >= 1
 NonOverlap_Object_5and_Object_7R: x_5 - 0.5 l_5 - x_7 - 0.5 l_7
   - 2.260000000894069 C514 >= -2.260000000894069
 NonOverlap_Object_5and_Object_7L: x_5 + 0.5 l_5 - x_7 + 0.5 l_7
   + 2.260000000894069 C515 <= 2.260000000894069
 NonOverlap_Object_5and_Object_7F: y_5 - 0.5 w_5 - y_7 - 0.5 w_7
   - 2.260000000894069 C516 >= -2.260000000894069
 NonOverlap_Object_5and_Object_7B: y_5 + 0.5 w_5 - y_7 + 0.5 w_7
   + 2.260000000894069 C517 <= 2.260000000894069
 NonOverlap_Object_5and_Object_7U: z_5 - 0.5 h_5 - z_7 - 0.5 h_7
   - 2.260000000894069 C518 >= -2.260000000894069
 NonOverlap_Object_5and_Object_7D: z_5 + 0.5 h_5 - z_7 + 0.5 h_7
   + 2.260000000894069 C519 <= 2.260000000894069
 NonOverlap_Object_5and_Object_7: C514 + C515 + C516 + C517 + C518 + C519
   >= 1
 NonOverlap_Object_6and_Object_7R: x_6 - 0.5 l_6 - x_7 - 0.5 l_7
   - 2.260000000894069 C520 >= -2.260000000894069
 NonOverlap_Object_6and_Object_7L: x_6 + 0.5 l_6 - x_7 + 0.5 l_7
   + 2.260000000894069 C521 <= 2.260000000894069
 NonOverlap_Object_6and_Object_7F: y_6 - 0.5 w_6 - y_7 - 0.5 w_7
   - 2.260000000894069 C522 >= -2.260000000894069
 NonOverlap_Object_6and_Object_7B: y_6 + 0.5 w_6 - y_7 + 0.5 w_7
   + 2.260000000894069 C523 <= 2.260000000894069
 NonOverlap_Object_6and_Object_7U: z_6 - 0.5 h_6 - z_7 - 0.5 h_7
   - 2.260000000894069 C524 >= -2.260000000894069
 NonOverlap_Object_6and_Object_7D: z_6 + 0.5 h_6 - z_7 + 0.5 h_7
   + 2.260000000894069 C525 <= 2.260000000894069
 NonOverlap_Object_6and_Object_7: C520 + C521 + C522 + C523 + C524 + C525
   >= 1
 NonOverlap_Object_0and_Obstacle_0L: x_0 + 0.5 l_0 + 2.260000000894069 C526
   <= 2.620000000894069
 NonOverlap_Object_0and_Obstacle_0R: x_0 - 0.5 l_0 - 2.260000000894069 C527
   >= -1.82000000089407
 NonOverlap_Object_0and_Obstacle_0B: y_0 + 0.5 w_0 + 2.260000000894069 C528
   <= 2.260000000894069
 NonOverlap_Object_0and_Obstacle_0F: y_0 - 0.5 w_0 - 2.260000000894069 C529
   >= -2.180000000894069
 NonOverlap_Object_0and_Obstacle_0D: z_0 + 0.5 h_0 + 2.260000000894069 C530
   <= 2.260000000894069
 NonOverlap_Object_0and_Obstacle_0U: z_0 - 0.5 h_0 - 2.260000000894069 C531
   >= -2.020000000894069
 NonOverlap_Object_0and_Obstacle_0: C526 + C527 + C528 + C529 + C530 + C531
   >= 1
 NonOverlap_Object_0and_Obstacle_1L: x_0 + 0.5 l_0 + 2.260000000894069 C532
   <= 2.260000000894069
 NonOverlap_Object_0and_Obstacle_1R: x_0 - 0.5 l_0 - 2.260000000894069 C533
   >= -2.060000000894069
 NonOverlap_Object_0and_Obstacle_1B: y_0 + 0.5 w_0 + 2.260000000894069 C534
   <= 2.760000000894069
 NonOverlap_Object_0and_Obstacle_1F: y_0 - 0.5 w_0 - 2.260000000894069 C535
   >= -1.56000000089407
 NonOverlap_Object_0and_Obstacle_1D: z_0 + 0.5 h_0 + 2.260000000894069 C536
   <= 2.310000000894069
 NonOverlap_Object_0and_Obstacle_1U: z_0 - 0.5 h_0 - 2.260000000894069 C537
   >= -2.010000000894069
 NonOverlap_Object_0and_Obstacle_1: C532 + C533 + C534 + C535 + C536 + C537
   >= 1
 NonOverlap_Object_0and_Obstacle_2L: x_0 + 0.5 l_0 + 2.260000000894069 C538
   <= 2.940000001117587
 NonOverlap_Object_0and_Obstacle_2R: x_0 - 0.5 l_0 - 2.260000000894069 C539
   >= -1.260000001117587
 NonOverlap_Object_0and_Obstacle_2B: y_0 + 0.5 w_0 + 2.260000000894069 C540
   <= 2.260000001117587
 NonOverlap_Object_0and_Obstacle_2F: y_0 - 0.5 w_0 - 2.260000000894069 C541
   >= -1.940000001117587
 NonOverlap_Object_0and_Obstacle_2D: z_0 + 0.5 h_0 + 2.260000000894069 C542
   <= 2.260000000894069
 NonOverlap_Object_0and_Obstacle_2U: z_0 - 0.5 h_0 - 2.260000000894069 C543
   >= -1.960000000894069
 NonOverlap_Object_0and_Obstacle_2: C538 + C539 + C540 + C541 + C542 + C543
   >= 1
 NonOverlap_Object_1and_Obstacle_0L: x_1 + 0.5 l_1 + 2.260000000894069 C544
   <= 2.620000000894069
 NonOverlap_Object_1and_Obstacle_0R: x_1 - 0.5 l_1 - 2.260000000894069 C545
   >= -1.82000000089407
 NonOverlap_Object_1and_Obstacle_0B: y_1 + 0.5 w_1 + 2.260000000894069 C546
   <= 2.260000000894069
 NonOverlap_Object_1and_Obstacle_0F: y_1 - 0.5 w_1 - 2.260000000894069 C547
   >= -2.180000000894069
 NonOverlap_Object_1and_Obstacle_0D: z_1 + 0.5 h_1 + 2.260000000894069 C548
   <= 2.260000000894069
 NonOverlap_Object_1and_Obstacle_0U: z_1 - 0.5 h_1 - 2.260000000894069 C549
   >= -2.020000000894069
 NonOverlap_Object_1and_Obstacle_0: C544 + C545 + C546 + C547 + C548 + C549
   >= 1
 NonOverlap_Object_1and_Obstacle_1L: x_1 + 0.5 l_1 + 2.260000000894069 C550
   <= 2.260000000894069
 NonOverlap_Object_1and_Obstacle_1R: x_1 - 0.5 l_1 - 2.260000000894069 C551
   >= -2.060000000894069
 NonOverlap_Object_1and_Obstacle_1B: y_1 + 0.5 w_1 + 2.260000000894069 C552
   <= 2.760000000894069
 NonOverlap_Object_1and_Obstacle_1F: y_1 - 0.5 w_1 - 2.260000000894069 C553
   >= -1.56000000089407
 NonOverlap_Object_1and_Obstacle_1D: z_1 + 0.5 h_1 + 2.260000000894069 C554
   <= 2.310000000894069
 NonOverlap_Object_1and_Obstacle_1U: z_1 - 0.5 h_1 - 2.260000000894069 C555
   >= -2.010000000894069
 NonOverlap_Object_1and_Obstacle_1: C550 + C551 + C552 + C553 + C554 + C555
   >= 1
 NonOverlap_Object_1and_Obstacle_2L: x_1 + 0.5 l_1 + 2.260000000894069 C556
   <= 2.940000001117587
 NonOverlap_Object_1and_Obstacle_2R: x_1 - 0.5 l_1 - 2.260000000894069 C557
   >= -1.260000001117587
 NonOverlap_Object_1and_Obstacle_2B: y_1 + 0.5 w_1 + 2.260000000894069 C558
   <= 2.260000001117587
 NonOverlap_Object_1and_Obstacle_2F: y_1 - 0.5 w_1 - 2.260000000894069 C559
   >= -1.940000001117587
 NonOverlap_Object_1and_Obstacle_2D: z_1 + 0.5 h_1 + 2.260000000894069 C560
   <= 2.260000000894069
 NonOverlap_Object_1and_Obstacle_2U: z_1 - 0.5 h_1 - 2.260000000894069 C561
   >= -1.960000000894069
 NonOverlap_Object_1and_Obstacle_2: C556 + C557 + C558 + C559 + C560 + C561
   >= 1
 NonOverlap_Object_2and_Obstacle_0L: x_2 + 0.5 l_2 + 2.260000000894069 C562
   <= 2.620000000894069
 NonOverlap_Object_2and_Obstacle_0R: x_2 - 0.5 l_2 - 2.260000000894069 C563
   >= -1.82000000089407
 NonOverlap_Object_2and_Obstacle_0B: y_2 + 0.5 w_2 + 2.260000000894069 C564
   <= 2.260000000894069
 NonOverlap_Object_2and_Obstacle_0F: y_2 - 0.5 w_2 - 2.260000000894069 C565
   >= -2.180000000894069
 NonOverlap_Object_2and_Obstacle_0D: z_2 + 0.5 h_2 + 2.260000000894069 C566
   <= 2.260000000894069
 NonOverlap_Object_2and_Obstacle_0U: z_2 - 0.5 h_2 - 2.260000000894069 C567
   >= -2.020000000894069
 NonOverlap_Object_2and_Obstacle_0: C562 + C563 + C564 + C565 + C566 + C567
   >= 1
 NonOverlap_Object_2and_Obstacle_1L: x_2 + 0.5 l_2 + 2.260000000894069 C568
   <= 2.260000000894069
 NonOverlap_Object_2and_Obstacle_1R: x_2 - 0.5 l_2 - 2.260000000894069 C569
   >= -2.060000000894069
 NonOverlap_Object_2and_Obstacle_1B: y_2 + 0.5 w_2 + 2.260000000894069 C570
   <= 2.760000000894069
 NonOverlap_Object_2and_Obstacle_1F: y_2 - 0.5 w_2 - 2.260000000894069 C571
   >= -1.56000000089407
 NonOverlap_Object_2and_Obstacle_1D: z_2 + 0.5 h_2 + 2.260000000894069 C572
   <= 2.310000000894069
 NonOverlap_Object_2and_Obstacle_1U: z_2 - 0.5 h_2 - 2.260000000894069 C573
   >= -2.010000000894069
 NonOverlap_Object_2and_Obstacle_1: C568 + C569 + C570 + C571 + C572 + C573
   >= 1
 NonOverlap_Object_2and_Obstacle_2L: x_2 + 0.5 l_2 + 2.260000000894069 C574
   <= 2.940000001117587
 NonOverlap_Object_2and_Obstacle_2R: x_2 - 0.5 l_2 - 2.260000000894069 C575
   >= -1.260000001117587
 NonOverlap_Object_2and_Obstacle_2B: y_2 + 0.5 w_2 + 2.260000000894069 C576
   <= 2.260000001117587
 NonOverlap_Object_2and_Obstacle_2F: y_2 - 0.5 w_2 - 2.260000000894069 C577
   >= -1.940000001117587
 NonOverlap_Object_2and_Obstacle_2D: z_2 + 0.5 h_2 + 2.260000000894069 C578
   <= 2.260000000894069
 NonOverlap_Object_2and_Obstacle_2U: z_2 - 0.5 h_2 - 2.260000000894069 C579
   >= -1.960000000894069
 NonOverlap_Object_2and_Obstacle_2: C574 + C575 + C576 + C577 + C578 + C579
   >= 1
 NonOverlap_Object_3and_Obstacle_0L: x_3 + 0.5 l_3 + 2.260000000894069 C580
   <= 2.620000000894069
 NonOverlap_Object_3and_Obstacle_0R: x_3 - 0.5 l_3 - 2.260000000894069 C581
   >= -1.82000000089407
 NonOverlap_Object_3and_Obstacle_0B: y_3 + 0.5 w_3 + 2.260000000894069 C582
   <= 2.260000000894069
 NonOverlap_Object_3and_Obstacle_0F: y_3 - 0.5 w_3 - 2.260000000894069 C583
   >= -2.180000000894069
 NonOverlap_Object_3and_Obstacle_0D: z_3 + 0.5 h_3 + 2.260000000894069 C584
   <= 2.260000000894069
 NonOverlap_Object_3and_Obstacle_0U: z_3 - 0.5 h_3 - 2.260000000894069 C585
   >= -2.020000000894069
 NonOverlap_Object_3and_Obstacle_0: C580 + C581 + C582 + C583 + C584 + C585
   >= 1
 NonOverlap_Object_3and_Obstacle_1L: x_3 + 0.5 l_3 + 2.260000000894069 C586
   <= 2.260000000894069
 NonOverlap_Object_3and_Obstacle_1R: x_3 - 0.5 l_3 - 2.260000000894069 C587
   >= -2.060000000894069
 NonOverlap_Object_3and_Obstacle_1B: y_3 + 0.5 w_3 + 2.260000000894069 C588
   <= 2.760000000894069
 NonOverlap_Object_3and_Obstacle_1F: y_3 - 0.5 w_3 - 2.260000000894069 C589
   >= -1.56000000089407
 NonOverlap_Object_3and_Obstacle_1D: z_3 + 0.5 h_3 + 2.260000000894069 C590
   <= 2.310000000894069
 NonOverlap_Object_3and_Obstacle_1U: z_3 - 0.5 h_3 - 2.260000000894069 C591
   >= -2.010000000894069
 NonOverlap_Object_3and_Obstacle_1: C586 + C587 + C588 + C589 + C590 + C591
   >= 1
 NonOverlap_Object_3and_Obstacle_2L: x_3 + 0.5 l_3 + 2.260000000894069 C592
   <= 2.940000001117587
 NonOverlap_Object_3and_Obstacle_2R: x_3 - 0.5 l_3 - 2.260000000894069 C593
   >= -1.260000001117587
 NonOverlap_Object_3and_Obstacle_2B: y_3 + 0.5 w_3 + 2.260000000894069 C594
   <= 2.260000001117587
 NonOverlap_Object_3and_Obstacle_2F: y_3 - 0.5 w_3 - 2.260000000894069 C595
   >= -1.940000001117587
 NonOverlap_Object_3and_Obstacle_2D: z_3 + 0.5 h_3 + 2.260000000894069 C596
   <= 2.260000000894069
 NonOverlap_Object_3and_Obstacle_2U: z_3 - 0.5 h_3 - 2.260000000894069 C597
   >= -1.960000000894069
 NonOverlap_Object_3and_Obstacle_2: C592 + C593 + C594 + C595 + C596 + C597
   >= 1
 NonOverlap_Object_4and_Obstacle_0L: x_4 + 0.5 l_4 + 2.260000000894069 C598
   <= 2.620000000894069
 NonOverlap_Object_4and_Obstacle_0R: x_4 - 0.5 l_4 - 2.260000000894069 C599
   >= -1.82000000089407
 NonOverlap_Object_4and_Obstacle_0B: y_4 + 0.5 w_4 + 2.260000000894069 C600
   <= 2.260000000894069
 NonOverlap_Object_4and_Obstacle_0F: y_4 - 0.5 w_4 - 2.260000000894069 C601
   >= -2.180000000894069
 NonOverlap_Object_4and_Obstacle_0D: z_4 + 0.5 h_4 + 2.260000000894069 C602
   <= 2.260000000894069
 NonOverlap_Object_4and_Obstacle_0U: z_4 - 0.5 h_4 - 2.260000000894069 C603
   >= -2.020000000894069
 NonOverlap_Object_4and_Obstacle_0: C598 + C599 + C600 + C601 + C602 + C603
   >= 1
 NonOverlap_Object_4and_Obstacle_1L: x_4 + 0.5 l_4 + 2.260000000894069 C604
   <= 2.260000000894069
 NonOverlap_Object_4and_Obstacle_1R: x_4 - 0.5 l_4 - 2.260000000894069 C605
   >= -2.060000000894069
 NonOverlap_Object_4and_Obstacle_1B: y_4 + 0.5 w_4 + 2.260000000894069 C606
   <= 2.760000000894069
 NonOverlap_Object_4and_Obstacle_1F: y_4 - 0.5 w_4 - 2.260000000894069 C607
   >= -1.56000000089407
 NonOverlap_Object_4and_Obstacle_1D: z_4 + 0.5 h_4 + 2.260000000894069 C608
   <= 2.310000000894069
 NonOverlap_Object_4and_Obstacle_1U: z_4 - 0.5 h_4 - 2.260000000894069 C609
   >= -2.010000000894069
 NonOverlap_Object_4and_Obstacle_1: C604 + C605 + C606 + C607 + C608 + C609
   >= 1
 NonOverlap_Object_4and_Obstacle_2L: x_4 + 0.5 l_4 + 2.260000000894069 C610
   <= 2.940000001117587
 NonOverlap_Object_4and_Obstacle_2R: x_4 - 0.5 l_4 - 2.260000000894069 C611
   >= -1.260000001117587
 NonOverlap_Object_4and_Obstacle_2B: y_4 + 0.5 w_4 + 2.260000000894069 C612
   <= 2.260000001117587
 NonOverlap_Object_4and_Obstacle_2F: y_4 - 0.5 w_4 - 2.260000000894069 C613
   >= -1.940000001117587
 NonOverlap_Object_4and_Obstacle_2D: z_4 + 0.5 h_4 + 2.260000000894069 C614
   <= 2.260000000894069
 NonOverlap_Object_4and_Obstacle_2U: z_4 - 0.5 h_4 - 2.260000000894069 C615
   >= -1.960000000894069
 NonOverlap_Object_4and_Obstacle_2: C610 + C611 + C612 + C613 + C614 + C615
   >= 1
 NonOverlap_Object_5and_Obstacle_0L: x_5 + 0.5 l_5 + 2.260000000894069 C616
   <= 2.620000000894069
 NonOverlap_Object_5and_Obstacle_0R: x_5 - 0.5 l_5 - 2.260000000894069 C617
   >= -1.82000000089407
 NonOverlap_Object_5and_Obstacle_0B: y_5 + 0.5 w_5 + 2.260000000894069 C618
   <= 2.260000000894069
 NonOverlap_Object_5and_Obstacle_0F: y_5 - 0.5 w_5 - 2.260000000894069 C619
   >= -2.180000000894069
 NonOverlap_Object_5and_Obstacle_0D: z_5 + 0.5 h_5 + 2.260000000894069 C620
   <= 2.260000000894069
 NonOverlap_Object_5and_Obstacle_0U: z_5 - 0.5 h_5 - 2.260000000894069 C621
   >= -2.020000000894069
 NonOverlap_Object_5and_Obstacle_0: C616 + C617 + C618 + C619 + C620 + C621
   >= 1
 NonOverlap_Object_5and_Obstacle_1L: x_5 + 0.5 l_5 + 2.260000000894069 C622
   <= 2.260000000894069
 NonOverlap_Object_5and_Obstacle_1R: x_5 - 0.5 l_5 - 2.260000000894069 C623
   >= -2.060000000894069
 NonOverlap_Object_5and_Obstacle_1B: y_5 + 0.5 w_5 + 2.260000000894069 C624
   <= 2.760000000894069
 NonOverlap_Object_5and_Obstacle_1F: y_5 - 0.5 w_5 - 2.260000000894069 C625
   >= -1.56000000089407
 NonOverlap_Object_5and_Obstacle_1D: z_5 + 0.5 h_5 + 2.260000000894069 C626
   <= 2.310000000894069
 NonOverlap_Object_5and_Obstacle_1U: z_5 - 0.5 h_5 - 2.260000000894069 C627
   >= -2.010000000894069
 NonOverlap_Object_5and_Obstacle_1: C622 + C623 + C624 + C625 + C626 + C627
   >= 1
 NonOverlap_Object_5and_Obstacle_2L: x_5 + 0.5 l_5 + 2.260000000894069 C628
   <= 2.940000001117587
 NonOverlap_Object_5and_Obstacle_2R: x_5 - 0.5 l_5 - 2.260000000894069 C629
   >= -1.260000001117587
 NonOverlap_Object_5and_Obstacle_2B: y_5 + 0.5 w_5 + 2.260000000894069 C630
   <= 2.260000001117587
 NonOverlap_Object_5and_Obstacle_2F: y_5 - 0.5 w_5 - 2.260000000894069 C631
   >= -1.940000001117587
 NonOverlap_Object_5and_Obstacle_2D: z_5 + 0.5 h_5 + 2.260000000894069 C632
   <= 2.260000000894069
 NonOverlap_Object_5and_Obstacle_2U: z_5 - 0.5 h_5 - 2.260000000894069 C633
   >= -1.960000000894069
 NonOverlap_Object_5and_Obstacle_2: C628 + C629 + C630 + C631 + C632 + C633
   >= 1
 NonOverlap_Object_6and_Obstacle_0L: x_6 + 0.5 l_6 + 2.260000000894069 C634
   <= 2.620000000894069
 NonOverlap_Object_6and_Obstacle_0R: x_6 - 0.5 l_6 - 2.260000000894069 C635
   >= -1.82000000089407
 NonOverlap_Object_6and_Obstacle_0B: y_6 + 0.5 w_6 + 2.260000000894069 C636
   <= 2.260000000894069
 NonOverlap_Object_6and_Obstacle_0F: y_6 - 0.5 w_6 - 2.260000000894069 C637
   >= -2.180000000894069
 NonOverlap_Object_6and_Obstacle_0D: z_6 + 0.5 h_6 + 2.260000000894069 C638
   <= 2.260000000894069
 NonOverlap_Object_6and_Obstacle_0U: z_6 - 0.5 h_6 - 2.260000000894069 C639
   >= -2.020000000894069
 NonOverlap_Object_6and_Obstacle_0: C634 + C635 + C636 + C637 + C638 + C639
   >= 1
 NonOverlap_Object_6and_Obstacle_1L: x_6 + 0.5 l_6 + 2.260000000894069 C640
   <= 2.260000000894069
 NonOverlap_Object_6and_Obstacle_1R: x_6 - 0.5 l_6 - 2.260000000894069 C641
   >= -2.060000000894069
 NonOverlap_Object_6and_Obstacle_1B: y_6 + 0.5 w_6 + 2.260000000894069 C642
   <= 2.760000000894069
 NonOverlap_Object_6and_Obstacle_1F: y_6 - 0.5 w_6 - 2.260000000894069 C643
   >= -1.56000000089407
 NonOverlap_Object_6and_Obstacle_1D: z_6 + 0.5 h_6 + 2.260000000894069 C644
   <= 2.310000000894069
 NonOverlap_Object_6and_Obstacle_1U: z_6 - 0.5 h_6 - 2.260000000894069 C645
   >= -2.010000000894069
 NonOverlap_Object_6and_Obstacle_1: C640 + C641 + C642 + C643 + C644 + C645
   >= 1
 NonOverlap_Object_6and_Obstacle_2L: x_6 + 0.5 l_6 + 2.260000000894069 C646
   <= 2.940000001117587
 NonOverlap_Object_6and_Obstacle_2R: x_6 - 0.5 l_6 - 2.260000000894069 C647
   >= -1.260000001117587
 NonOverlap_Object_6and_Obstacle_2B: y_6 + 0.5 w_6 + 2.260000000894069 C648
   <= 2.260000001117587
 NonOverlap_Object_6and_Obstacle_2F: y_6 - 0.5 w_6 - 2.260000000894069 C649
   >= -1.940000001117587
 NonOverlap_Object_6and_Obstacle_2D: z_6 + 0.5 h_6 + 2.260000000894069 C650
   <= 2.260000000894069
 NonOverlap_Object_6and_Obstacle_2U: z_6 - 0.5 h_6 - 2.260000000894069 C651
   >= -1.960000000894069
 NonOverlap_Object_6and_Obstacle_2: C646 + C647 + C648 + C649 + C650 + C651
   >= 1
 NonOverlap_Object_7and_Obstacle_0L: x_7 + 0.5 l_7 + 2.260000000894069 C652
   <= 2.620000000894069
 NonOverlap_Object_7and_Obstacle_0R: x_7 - 0.5 l_7 - 2.260000000894069 C653
   >= -1.82000000089407
 NonOverlap_Object_7and_Obstacle_0B: y_7 + 0.5 w_7 + 2.260000000894069 C654
   <= 2.260000000894069
 NonOverlap_Object_7and_Obstacle_0F: y_7 - 0.5 w_7 - 2.260000000894069 C655
   >= -2.180000000894069
 NonOverlap_Object_7and_Obstacle_0D: z_7 + 0.5 h_7 + 2.260000000894069 C656
   <= 2.260000000894069
 NonOverlap_Object_7and_Obstacle_0U: z_7 - 0.5 h_7 - 2.260000000894069 C657
   >= -2.020000000894069
 NonOverlap_Object_7and_Obstacle_0: C652 + C653 + C654 + C655 + C656 + C657
   >= 1
 NonOverlap_Object_7and_Obstacle_1L: x_7 + 0.5 l_7 + 2.260000000894069 C658
   <= 2.260000000894069
 NonOverlap_Object_7and_Obstacle_1R: x_7 - 0.5 l_7 - 2.260000000894069 C659
   >= -2.060000000894069
 NonOverlap_Object_7and_Obstacle_1B: y_7 + 0.5 w_7 + 2.260000000894069 C660
   <= 2.760000000894069
 NonOverlap_Object_7and_Obstacle_1F: y_7 - 0.5 w_7 - 2.260000000894069 C661
   >= -1.56000000089407
 NonOverlap_Object_7and_Obstacle_1D: z_7 + 0.5 h_7 + 2.260000000894069 C662
   <= 2.310000000894069
 NonOverlap_Object_7and_Obstacle_1U: z_7 - 0.5 h_7 - 2.260000000894069 C663
   >= -2.010000000894069
 NonOverlap_Object_7and_Obstacle_1: C658 + C659 + C660 + C661 + C662 + C663
   >= 1
 NonOverlap_Object_7and_Obstacle_2L: x_7 + 0.5 l_7 + 2.260000000894069 C664
   <= 2.940000001117587
 NonOverlap_Object_7and_Obstacle_2R: x_7 - 0.5 l_7 - 2.260000000894069 C665
   >= -1.260000001117587
 NonOverlap_Object_7and_Obstacle_2B: y_7 + 0.5 w_7 + 2.260000000894069 C666
   <= 2.260000001117587
 NonOverlap_Object_7and_Obstacle_2F: y_7 - 0.5 w_7 - 2.260000000894069 C667
   >= -1.940000001117587
 NonOverlap_Object_7and_Obstacle_2D: z_7 + 0.5 h_7 + 2.260000000894069 C668
   <= 2.260000000894069
 NonOverlap_Object_7and_Obstacle_2U: z_7 - 0.5 h_7 - 2.260000000894069 C669
   >= -1.960000000894069
 NonOverlap_Object_7and_Obstacle_2: C664 + C665 + C666 + C667 + C668 + C669
   >= 1
 Boundary_Object_0_Right_eq: x_0 + 0.5 l_0 = 0.99
 Boundary_Object_0_Right_ieq: y_0 >= 0.31
 Boundary_Object_0_Right_ieqq: y_0 <= 0.99
 Boundary_Object_4_Left_eq: x_4 - 0.5 l_4 = 0.01
 Boundary_Object_4_Left_ieq: y_4 >= 0.01
 Boundary_Object_4_Left_ieqq: y_4 <= 0.99
 TopLeft_Corner_of_Object_3eqa: C670 = 1
 TopLeft_Corner_of_Object_3eqb: x_3 - 0.5 l_3 - 0.01 C670 = 0
 TopLeft_Corner_of_Object_3eqc: y_3 + 0.5 w_3 - 0.99 C670 = 0
Bounds
 0.00999999977648258 <= x_0 <= 0.9900000002235174
 0.00999999977648258 <= y_0 <= 0.9900000002235174
 l_0 <= 0.9800000004470348
 w_0 <= 0.9800000004470348
 z_0 <= 0.3
 h_0 <= 0.3
 0.00999999977648258 <= x_1 <= 0.9900000002235174
 0.00999999977648258 <= y_1 <= 0.9900000002235174
 l_1 <= 0.9800000004470348
 w_1 <= 0.9800000004470348
 z_1 <= 0.3
 h_1 <= 0.3
 0.00999999977648258 <= x_2 <= 0.9900000002235174
 0.00999999977648258 <= y_2 <= 0.9900000002235174
 l_2 <= 0.9800000004470348
 w_2 <= 0.9800000004470348
 z_2 <= 0.3
 h_2 <= 0.3
 0.00999999977648258 <= x_3 <= 0.9900000002235174
 0.00999999977648258 <= y_3 <= 0.9900000002235174
 l_3 <= 0.9800000004470348
 w_3 <= 0.9800000004470348
 z_3 <= 0.3
 h_3 <= 0.3
 0.00999999977648258 <= x_4 <= 0.9900000002235174
 0.00999999977648258 <= y_4 <= 0.9900000002235174
 l_4 <= 0.9800000004470348
 w_4 <= 0.9800000004470348
 z_4 <= 0.3
 h_4 <= 0.3
 0.00999999977648258 <= x_5 <= 0.9900000002235174
 0.00999999977648258 <= y_5 <= 0.9900000002235174
 l_5 <= 0.9800000004470348
 w_5 <= 0.9800000004470348
 z_5 <= 0.3
 h_5 <= 0.3
 0.00999999977648258 <= x_6 <= 0.9900000002235174
 0.00999999977648258 <= y_6 <= 0.9900000002235174
 l_6 <= 0.9800000004470348
 w_6 <= 0.9800000004470348
 z_6 <= 0.3
 h_6 <= 0.3
 0.00999999977648258 <= x_7 <= 0.9900000002235174
 0.00999999977648258 <= y_7 <= 0.9900000002235174
 l_7 <= 0.9800000004470348
 w_7 <= 0.9800000004470348
 z_7 <= 0.3
 h_7 <= 0.3
 Constant = 1
Binaries
 C396 C397 C398 C399 C400 C401 C402 C403 C404 C405 C406 C407 C408 C409 C410
 C411 C412 C413 C414 C415 C416 C417 C418 C419 C420 C421 C422 C423 C424 C425
 C426 C427 C428 C429 C430 C431 C432 C433 C434 C435 C436 C437 C438 C439 C440
 C441 C442 C443 C444 C445 C446 C447 C448 C449 C450 C451 C452 C453 C454 C455
 C456 C457 C458 C459 C460 C461 C462 C463 C464 C465 C466 C467 C468 C469 C470
 C471 C472 C473 C474 C475 C476 C477 C478 C479 C480 C481 C482 C483 C484 C485
 C486 C487 C488 C489 C490 C491 C492 C493 C494 C495 C496 C497 C498 C499 C500
 C501 C502 C503 C504 C505 C506 C507 C508 C509 C510 C511 C512 C513 C514 C515
 C516 C517 C518 C519 C520 C521 C522 C523 C524 C525 C526 C527 C528 C529 C530
 C531 C532 C533 C534 C535 C536 C537 C538 C539 C540 C541 C542 C543 C544 C545
 C546 C547 C548 C549 C550 C551 C552 C553 C554 C555 C556 C557 C558 C559 C560
 C561 C562 C563 C564 C565 C566 C567 C568 C569 C570 C571 C572 C573 C574 C575
 C576 C577 C578 C579 C580 C581 C582 C583 C584 C585 C586 C587 C588 C589 C590
 C591 C592 C593 C594 C595 C596 C597 C598 C599 C600 C601 C602 C603 C604 C605
 C606 C607 C608 C609 C610 C611 C612 C613 C614 C615 C616 C617 C618 C619 C620
 C621 C622 C623 C624 C625 C626 C627 C628 C629 C630 C631 C632 C633 C634 C635
 C636 C637 C638 C639 C640 C641 C642 C643 C644 C645 C646 C647 C648 C649 C650
 C651 C652 C653 C654 C655 C656 C657 C658 C659 C660 C661 C662 C663 C664 C665
 C666 C667 C668 C669 C670
End
