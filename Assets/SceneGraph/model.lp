\ LP format - for model browsing. Use MPS format to capture full model detail.
\ Signature: 0x44296d3af0dfd45e
Minimize
  - 0.4 x_0 - 0.6 l_0 - 0.4 w_0 - 0.2 h_0 - 0.2 l_1 - 0.2 w_1 - 0.16 h_1
   - 0.2 l_2 - 0.2 w_2 - 0.16 h_2 + 0.4 x_3 - 0.4 y_3 - 0.6 l_3 - 0.1 w_3
   - 0.4 h_3 + 0.02 x_4 + 0.4 y_4 - 0.29 l_4 - 0.3 w_4 - 0.24 h_4
   - 0.02 x_5 - 0.15 l_5 - 0.16 w_5 - 0.16 h_5 - 1.2 y_6 - 0.08 l_6
   - 0.4 w_6 - 0.5 z_6 - 0.08 h_6 - 1.2 x_7 - 0.4 y_7 - 0.04 l_7 - 0.04 w_7
   - 0.6 z_7 - 0.04 h_7 + 2.601300000876188 Constant + [ 2 x_0 ^2
   - 2 x_0 * l_0 - 4 x_0 * x_3 - 2 x_0 * l_3 + 4 y_0 ^2 - 4 y_0 * y_2
   - 2 y_0 * w_2 - 4 y_0 * y_1 + 2 y_0 * w_1 - 2 l_0 * w_0 + 2.5 l_0 ^2
   + 2 l_0 * x_3 + l_0 * l_3 + 3 w_0 ^2 + 2 w_0 * y_2 + w_0 * w_2
   - 2 w_0 * y_1 + w_0 * w_1 + 2 h_0 ^2 + 2 y_1 ^2 - 2 y_1 * w_1
   - 2 l_1 * w_1 + 2 l_1 ^2 + 2.5 w_1 ^2 + 2 h_1 ^2 + 2 x_2 ^2
   - 2 x_2 * l_2 - 4 x_2 * x_3 - 2 x_2 * l_3 + 2 y_2 ^2 + 2 y_2 * w_2
   - 2 l_2 * w_2 + 2.5 l_2 ^2 + 2 l_2 * x_3 + l_2 * l_3 + 2.5 w_2 ^2
   + 2 h_2 ^2 + 6 x_3 ^2 + 2 x_3 * l_3 - 4 x_3 * x_4 - 2 x_3 * l_4
   + 2 y_3 ^2 - 2 y_3 * w_3 - 4 y_3 * y_4 - 2 y_3 * w_4 - 2 l_3 * w_3
   + 3.5 l_3 ^2 + 2 l_3 * x_4 + l_3 * l_4 + 2.5 w_3 ^2 + 2 w_3 * y_4
   + w_3 * w_4 + 2 h_3 ^2 + 4 x_4 ^2 + 4 x_4 * l_4 - 4 x_4 * x_5
   + 2 x_4 * l_5 + 2 y_4 ^2 + 2 y_4 * w_4 - 2 l_4 * w_4 + 3 l_4 ^2
   - 2 l_4 * x_5 + l_4 * l_5 + 2.5 w_4 ^2 - 4 z_4 * z_7 + 2 z_4 * h_7
   + 2 z_4 ^2 + 2 z_4 * h_4 + 2.5 h_4 ^2 - 2 h_4 * z_7 + h_4 * h_7
   + 2 x_5 ^2 - 2 x_5 * l_5 - 2 l_5 * w_5 + 2.5 l_5 ^2 + 2 w_5 ^2
   + 2 h_5 ^2 + 2 x_6 ^2 + 2 y_6 ^2 - 2 l_6 * w_6 + 2 l_6 ^2 + 2 w_6 ^2
   + 2 z_6 ^2 + 2 h_6 ^2 + 2 x_7 ^2 + 2 y_7 ^2 - 2 l_7 * w_7 + 2 l_7 ^2
   + 2 w_7 ^2 + 4 z_7 ^2 - 2 z_7 * h_7 + 2.5 h_7 ^2 ] / 2 
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
 Size_Tolerance_Object_6_w_min: w_6 >= 0.2
 Size_Tolerance_Object_6_w_max: w_6 <= 0.2
 Size_Tolerance_Object_6_h_min: h_6 >= 0.04
 Size_Tolerance_Object_6_h_max: h_6 <= 0.04
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
 Object_0_FrontOf_Object_2: y_0 - 0.5 w_0 - y_2 - 0.5 w_2 >= 0
 Object_1_AlignWith_Object_0: - x_0 - 0.5 l_0 + x_1 + 0.5 l_1 = 0
 Object_1_FrontOf_Object_0: - y_0 - 0.5 w_0 + y_1 - 0.5 w_1 >= 0
 Object_2_AlignWith_Object_0: - x_0 - 0.5 l_0 + x_2 + 0.5 l_2 = 0
 Object_3_LeftOf_Object_0: - x_0 + 0.5 l_0 + x_3 + 0.5 l_3 <= 0
 Object_3_LeftOf_Object_2: - x_2 + 0.5 l_2 + x_3 + 0.5 l_3 <= 0
 Object_3_FrontOf_Object_4: y_3 - 0.5 w_3 - y_4 - 0.5 w_4 >= 0
 Object_4_LeftOf_Object_3: - x_3 + 0.5 l_3 + x_4 + 0.5 l_4 <= 0
 Object_4_LeftOf_Object_5: x_4 + 0.5 l_4 - x_5 + 0.5 l_5 <= 0
 Object_5_CloseBy_Object_4ieqa: - x_4 - 0.5 l_4 + x_5 - 0.5 l_5 + 0.08 C48
   <= 0
 Object_5_CloseBy_Object_4ieqb: - x_4 + 0.5 l_4 + x_5 + 0.5 l_5 - 0.08 C48
   >= 0
 Object_5_CloseBy_Object_4ieqc: - y_4 - 0.5 w_4 + y_5 - 0.5 w_5 - 0.08 C48
   <= -0.08
 Object_5_CloseBy_Object_4ieqd: - y_4 + 0.5 w_4 + y_5 + 0.5 w_5 + 0.08 C48
   >= 0.08
 Object_7_Above_Object_4: - z_4 - 0.5 h_4 + z_7 - 0.5 h_7 >= 0
 Object_7_Above_Object_4ieqa: - x_4 + 0.5 l_4 + x_7 - 0.5 l_7 >= 0
 Object_7_Above_Object_4ieqb: - x_4 - 0.5 l_4 + x_7 + 0.5 l_7 <= 0
 Object_7_Above_Object_4ieqc: - y_4 + 0.5 w_4 + y_7 - 0.5 w_7 >= 0
 Object_7_Above_Object_4ieqd: - y_4 - 0.5 w_4 + y_7 + 0.5 w_7 <= 0
 NonOverlap_Object_0and_Object_5R: x_0 - 0.5 l_0 - x_5 - 0.5 l_5
   - 2.260000000894069 C49 >= -2.260000000894069
 NonOverlap_Object_0and_Object_5L: x_0 + 0.5 l_0 - x_5 + 0.5 l_5
   + 2.260000000894069 C50 <= 2.260000000894069
 NonOverlap_Object_0and_Object_5F: y_0 - 0.5 w_0 - y_5 - 0.5 w_5
   - 2.260000000894069 C51 >= -2.260000000894069
 NonOverlap_Object_0and_Object_5B: y_0 + 0.5 w_0 - y_5 + 0.5 w_5
   + 2.260000000894069 C52 <= 2.260000000894069
 NonOverlap_Object_0and_Object_5U: z_0 - 0.5 h_0 - z_5 - 0.5 h_5
   - 2.260000000894069 C53 >= -2.260000000894069
 NonOverlap_Object_0and_Object_5D: z_0 + 0.5 h_0 - z_5 + 0.5 h_5
   + 2.260000000894069 C54 <= 2.260000000894069
 NonOverlap_Object_0and_Object_5: C49 + C50 + C51 + C52 + C53 + C54 >= 1
 NonOverlap_Object_0and_Object_6R: x_0 - 0.5 l_0 - x_6 - 0.5 l_6
   - 2.260000000894069 C55 >= -2.260000000894069
 NonOverlap_Object_0and_Object_6L: x_0 + 0.5 l_0 - x_6 + 0.5 l_6
   + 2.260000000894069 C56 <= 2.260000000894069
 NonOverlap_Object_0and_Object_6F: y_0 - 0.5 w_0 - y_6 - 0.5 w_6
   - 2.260000000894069 C57 >= -2.260000000894069
 NonOverlap_Object_0and_Object_6B: y_0 + 0.5 w_0 - y_6 + 0.5 w_6
   + 2.260000000894069 C58 <= 2.260000000894069
 NonOverlap_Object_0and_Object_6U: z_0 - 0.5 h_0 - z_6 - 0.5 h_6
   - 2.260000000894069 C59 >= -2.260000000894069
 NonOverlap_Object_0and_Object_6D: z_0 + 0.5 h_0 - z_6 + 0.5 h_6
   + 2.260000000894069 C60 <= 2.260000000894069
 NonOverlap_Object_0and_Object_6: C55 + C56 + C57 + C58 + C59 + C60 >= 1
 NonOverlap_Object_0and_Object_7R: x_0 - 0.5 l_0 - x_7 - 0.5 l_7
   - 2.260000000894069 C61 >= -2.260000000894069
 NonOverlap_Object_0and_Object_7L: x_0 + 0.5 l_0 - x_7 + 0.5 l_7
   + 2.260000000894069 C62 <= 2.260000000894069
 NonOverlap_Object_0and_Object_7F: y_0 - 0.5 w_0 - y_7 - 0.5 w_7
   - 2.260000000894069 C63 >= -2.260000000894069
 NonOverlap_Object_0and_Object_7B: y_0 + 0.5 w_0 - y_7 + 0.5 w_7
   + 2.260000000894069 C64 <= 2.260000000894069
 NonOverlap_Object_0and_Object_7U: z_0 - 0.5 h_0 - z_7 - 0.5 h_7
   - 2.260000000894069 C65 >= -2.260000000894069
 NonOverlap_Object_0and_Object_7D: z_0 + 0.5 h_0 - z_7 + 0.5 h_7
   + 2.260000000894069 C66 <= 2.260000000894069
 NonOverlap_Object_0and_Object_7: C61 + C62 + C63 + C64 + C65 + C66 >= 1
 NonOverlap_Object_1and_Object_3R: x_1 - 0.5 l_1 - x_3 - 0.5 l_3
   - 2.260000000894069 C67 >= -2.260000000894069
 NonOverlap_Object_1and_Object_3L: x_1 + 0.5 l_1 - x_3 + 0.5 l_3
   + 2.260000000894069 C68 <= 2.260000000894069
 NonOverlap_Object_1and_Object_3F: y_1 - 0.5 w_1 - y_3 - 0.5 w_3
   - 2.260000000894069 C69 >= -2.260000000894069
 NonOverlap_Object_1and_Object_3B: y_1 + 0.5 w_1 - y_3 + 0.5 w_3
   + 2.260000000894069 C70 <= 2.260000000894069
 NonOverlap_Object_1and_Object_3U: z_1 - 0.5 h_1 - z_3 - 0.5 h_3
   - 2.260000000894069 C71 >= -2.260000000894069
 NonOverlap_Object_1and_Object_3D: z_1 + 0.5 h_1 - z_3 + 0.5 h_3
   + 2.260000000894069 C72 <= 2.260000000894069
 NonOverlap_Object_1and_Object_3: C67 + C68 + C69 + C70 + C71 + C72 >= 1
 NonOverlap_Object_1and_Object_4R: x_1 - 0.5 l_1 - x_4 - 0.5 l_4
   - 2.260000000894069 C73 >= -2.260000000894069
 NonOverlap_Object_1and_Object_4L: x_1 + 0.5 l_1 - x_4 + 0.5 l_4
   + 2.260000000894069 C74 <= 2.260000000894069
 NonOverlap_Object_1and_Object_4F: y_1 - 0.5 w_1 - y_4 - 0.5 w_4
   - 2.260000000894069 C75 >= -2.260000000894069
 NonOverlap_Object_1and_Object_4B: y_1 + 0.5 w_1 - y_4 + 0.5 w_4
   + 2.260000000894069 C76 <= 2.260000000894069
 NonOverlap_Object_1and_Object_4U: z_1 - 0.5 h_1 - z_4 - 0.5 h_4
   - 2.260000000894069 C77 >= -2.260000000894069
 NonOverlap_Object_1and_Object_4D: z_1 + 0.5 h_1 - z_4 + 0.5 h_4
   + 2.260000000894069 C78 <= 2.260000000894069
 NonOverlap_Object_1and_Object_4: C73 + C74 + C75 + C76 + C77 + C78 >= 1
 NonOverlap_Object_1and_Object_5R: x_1 - 0.5 l_1 - x_5 - 0.5 l_5
   - 2.260000000894069 C79 >= -2.260000000894069
 NonOverlap_Object_1and_Object_5L: x_1 + 0.5 l_1 - x_5 + 0.5 l_5
   + 2.260000000894069 C80 <= 2.260000000894069
 NonOverlap_Object_1and_Object_5F: y_1 - 0.5 w_1 - y_5 - 0.5 w_5
   - 2.260000000894069 C81 >= -2.260000000894069
 NonOverlap_Object_1and_Object_5B: y_1 + 0.5 w_1 - y_5 + 0.5 w_5
   + 2.260000000894069 C82 <= 2.260000000894069
 NonOverlap_Object_1and_Object_5U: z_1 - 0.5 h_1 - z_5 - 0.5 h_5
   - 2.260000000894069 C83 >= -2.260000000894069
 NonOverlap_Object_1and_Object_5D: z_1 + 0.5 h_1 - z_5 + 0.5 h_5
   + 2.260000000894069 C84 <= 2.260000000894069
 NonOverlap_Object_1and_Object_5: C79 + C80 + C81 + C82 + C83 + C84 >= 1
 NonOverlap_Object_1and_Object_6R: x_1 - 0.5 l_1 - x_6 - 0.5 l_6
   - 2.260000000894069 C85 >= -2.260000000894069
 NonOverlap_Object_1and_Object_6L: x_1 + 0.5 l_1 - x_6 + 0.5 l_6
   + 2.260000000894069 C86 <= 2.260000000894069
 NonOverlap_Object_1and_Object_6F: y_1 - 0.5 w_1 - y_6 - 0.5 w_6
   - 2.260000000894069 C87 >= -2.260000000894069
 NonOverlap_Object_1and_Object_6B: y_1 + 0.5 w_1 - y_6 + 0.5 w_6
   + 2.260000000894069 C88 <= 2.260000000894069
 NonOverlap_Object_1and_Object_6U: z_1 - 0.5 h_1 - z_6 - 0.5 h_6
   - 2.260000000894069 C89 >= -2.260000000894069
 NonOverlap_Object_1and_Object_6D: z_1 + 0.5 h_1 - z_6 + 0.5 h_6
   + 2.260000000894069 C90 <= 2.260000000894069
 NonOverlap_Object_1and_Object_6: C85 + C86 + C87 + C88 + C89 + C90 >= 1
 NonOverlap_Object_1and_Object_7R: x_1 - 0.5 l_1 - x_7 - 0.5 l_7
   - 2.260000000894069 C91 >= -2.260000000894069
 NonOverlap_Object_1and_Object_7L: x_1 + 0.5 l_1 - x_7 + 0.5 l_7
   + 2.260000000894069 C92 <= 2.260000000894069
 NonOverlap_Object_1and_Object_7F: y_1 - 0.5 w_1 - y_7 - 0.5 w_7
   - 2.260000000894069 C93 >= -2.260000000894069
 NonOverlap_Object_1and_Object_7B: y_1 + 0.5 w_1 - y_7 + 0.5 w_7
   + 2.260000000894069 C94 <= 2.260000000894069
 NonOverlap_Object_1and_Object_7U: z_1 - 0.5 h_1 - z_7 - 0.5 h_7
   - 2.260000000894069 C95 >= -2.260000000894069
 NonOverlap_Object_1and_Object_7D: z_1 + 0.5 h_1 - z_7 + 0.5 h_7
   + 2.260000000894069 C96 <= 2.260000000894069
 NonOverlap_Object_1and_Object_7: C91 + C92 + C93 + C94 + C95 + C96 >= 1
 NonOverlap_Object_2and_Object_5R: x_2 - 0.5 l_2 - x_5 - 0.5 l_5
   - 2.260000000894069 C97 >= -2.260000000894069
 NonOverlap_Object_2and_Object_5L: x_2 + 0.5 l_2 - x_5 + 0.5 l_5
   + 2.260000000894069 C98 <= 2.260000000894069
 NonOverlap_Object_2and_Object_5F: y_2 - 0.5 w_2 - y_5 - 0.5 w_5
   - 2.260000000894069 C99 >= -2.260000000894069
 NonOverlap_Object_2and_Object_5B: y_2 + 0.5 w_2 - y_5 + 0.5 w_5
   + 2.260000000894069 C100 <= 2.260000000894069
 NonOverlap_Object_2and_Object_5U: z_2 - 0.5 h_2 - z_5 - 0.5 h_5
   - 2.260000000894069 C101 >= -2.260000000894069
 NonOverlap_Object_2and_Object_5D: z_2 + 0.5 h_2 - z_5 + 0.5 h_5
   + 2.260000000894069 C102 <= 2.260000000894069
 NonOverlap_Object_2and_Object_5: C97 + C98 + C99 + C100 + C101 + C102
   >= 1
 NonOverlap_Object_2and_Object_6R: x_2 - 0.5 l_2 - x_6 - 0.5 l_6
   - 2.260000000894069 C103 >= -2.260000000894069
 NonOverlap_Object_2and_Object_6L: x_2 + 0.5 l_2 - x_6 + 0.5 l_6
   + 2.260000000894069 C104 <= 2.260000000894069
 NonOverlap_Object_2and_Object_6F: y_2 - 0.5 w_2 - y_6 - 0.5 w_6
   - 2.260000000894069 C105 >= -2.260000000894069
 NonOverlap_Object_2and_Object_6B: y_2 + 0.5 w_2 - y_6 + 0.5 w_6
   + 2.260000000894069 C106 <= 2.260000000894069
 NonOverlap_Object_2and_Object_6U: z_2 - 0.5 h_2 - z_6 - 0.5 h_6
   - 2.260000000894069 C107 >= -2.260000000894069
 NonOverlap_Object_2and_Object_6D: z_2 + 0.5 h_2 - z_6 + 0.5 h_6
   + 2.260000000894069 C108 <= 2.260000000894069
 NonOverlap_Object_2and_Object_6: C103 + C104 + C105 + C106 + C107 + C108
   >= 1
 NonOverlap_Object_2and_Object_7R: x_2 - 0.5 l_2 - x_7 - 0.5 l_7
   - 2.260000000894069 C109 >= -2.260000000894069
 NonOverlap_Object_2and_Object_7L: x_2 + 0.5 l_2 - x_7 + 0.5 l_7
   + 2.260000000894069 C110 <= 2.260000000894069
 NonOverlap_Object_2and_Object_7F: y_2 - 0.5 w_2 - y_7 - 0.5 w_7
   - 2.260000000894069 C111 >= -2.260000000894069
 NonOverlap_Object_2and_Object_7B: y_2 + 0.5 w_2 - y_7 + 0.5 w_7
   + 2.260000000894069 C112 <= 2.260000000894069
 NonOverlap_Object_2and_Object_7U: z_2 - 0.5 h_2 - z_7 - 0.5 h_7
   - 2.260000000894069 C113 >= -2.260000000894069
 NonOverlap_Object_2and_Object_7D: z_2 + 0.5 h_2 - z_7 + 0.5 h_7
   + 2.260000000894069 C114 <= 2.260000000894069
 NonOverlap_Object_2and_Object_7: C109 + C110 + C111 + C112 + C113 + C114
   >= 1
 NonOverlap_Object_3and_Object_5R: x_3 - 0.5 l_3 - x_5 - 0.5 l_5
   - 2.260000000894069 C115 >= -2.260000000894069
 NonOverlap_Object_3and_Object_5L: x_3 + 0.5 l_3 - x_5 + 0.5 l_5
   + 2.260000000894069 C116 <= 2.260000000894069
 NonOverlap_Object_3and_Object_5F: y_3 - 0.5 w_3 - y_5 - 0.5 w_5
   - 2.260000000894069 C117 >= -2.260000000894069
 NonOverlap_Object_3and_Object_5B: y_3 + 0.5 w_3 - y_5 + 0.5 w_5
   + 2.260000000894069 C118 <= 2.260000000894069
 NonOverlap_Object_3and_Object_5U: z_3 - 0.5 h_3 - z_5 - 0.5 h_5
   - 2.260000000894069 C119 >= -2.260000000894069
 NonOverlap_Object_3and_Object_5D: z_3 + 0.5 h_3 - z_5 + 0.5 h_5
   + 2.260000000894069 C120 <= 2.260000000894069
 NonOverlap_Object_3and_Object_5: C115 + C116 + C117 + C118 + C119 + C120
   >= 1
 NonOverlap_Object_3and_Object_6R: x_3 - 0.5 l_3 - x_6 - 0.5 l_6
   - 2.260000000894069 C121 >= -2.260000000894069
 NonOverlap_Object_3and_Object_6L: x_3 + 0.5 l_3 - x_6 + 0.5 l_6
   + 2.260000000894069 C122 <= 2.260000000894069
 NonOverlap_Object_3and_Object_6F: y_3 - 0.5 w_3 - y_6 - 0.5 w_6
   - 2.260000000894069 C123 >= -2.260000000894069
 NonOverlap_Object_3and_Object_6B: y_3 + 0.5 w_3 - y_6 + 0.5 w_6
   + 2.260000000894069 C124 <= 2.260000000894069
 NonOverlap_Object_3and_Object_6U: z_3 - 0.5 h_3 - z_6 - 0.5 h_6
   - 2.260000000894069 C125 >= -2.260000000894069
 NonOverlap_Object_3and_Object_6D: z_3 + 0.5 h_3 - z_6 + 0.5 h_6
   + 2.260000000894069 C126 <= 2.260000000894069
 NonOverlap_Object_3and_Object_6: C121 + C122 + C123 + C124 + C125 + C126
   >= 1
 NonOverlap_Object_3and_Object_7R: x_3 - 0.5 l_3 - x_7 - 0.5 l_7
   - 2.260000000894069 C127 >= -2.260000000894069
 NonOverlap_Object_3and_Object_7L: x_3 + 0.5 l_3 - x_7 + 0.5 l_7
   + 2.260000000894069 C128 <= 2.260000000894069
 NonOverlap_Object_3and_Object_7F: y_3 - 0.5 w_3 - y_7 - 0.5 w_7
   - 2.260000000894069 C129 >= -2.260000000894069
 NonOverlap_Object_3and_Object_7B: y_3 + 0.5 w_3 - y_7 + 0.5 w_7
   + 2.260000000894069 C130 <= 2.260000000894069
 NonOverlap_Object_3and_Object_7U: z_3 - 0.5 h_3 - z_7 - 0.5 h_7
   - 2.260000000894069 C131 >= -2.260000000894069
 NonOverlap_Object_3and_Object_7D: z_3 + 0.5 h_3 - z_7 + 0.5 h_7
   + 2.260000000894069 C132 <= 2.260000000894069
 NonOverlap_Object_3and_Object_7: C127 + C128 + C129 + C130 + C131 + C132
   >= 1
 NonOverlap_Object_4and_Object_6R: x_4 - 0.5 l_4 - x_6 - 0.5 l_6
   - 2.260000000894069 C133 >= -2.260000000894069
 NonOverlap_Object_4and_Object_6L: x_4 + 0.5 l_4 - x_6 + 0.5 l_6
   + 2.260000000894069 C134 <= 2.260000000894069
 NonOverlap_Object_4and_Object_6F: y_4 - 0.5 w_4 - y_6 - 0.5 w_6
   - 2.260000000894069 C135 >= -2.260000000894069
 NonOverlap_Object_4and_Object_6B: y_4 + 0.5 w_4 - y_6 + 0.5 w_6
   + 2.260000000894069 C136 <= 2.260000000894069
 NonOverlap_Object_4and_Object_6U: z_4 - 0.5 h_4 - z_6 - 0.5 h_6
   - 2.260000000894069 C137 >= -2.260000000894069
 NonOverlap_Object_4and_Object_6D: z_4 + 0.5 h_4 - z_6 + 0.5 h_6
   + 2.260000000894069 C138 <= 2.260000000894069
 NonOverlap_Object_4and_Object_6: C133 + C134 + C135 + C136 + C137 + C138
   >= 1
 NonOverlap_Object_5and_Object_6R: x_5 - 0.5 l_5 - x_6 - 0.5 l_6
   - 2.260000000894069 C139 >= -2.260000000894069
 NonOverlap_Object_5and_Object_6L: x_5 + 0.5 l_5 - x_6 + 0.5 l_6
   + 2.260000000894069 C140 <= 2.260000000894069
 NonOverlap_Object_5and_Object_6F: y_5 - 0.5 w_5 - y_6 - 0.5 w_6
   - 2.260000000894069 C141 >= -2.260000000894069
 NonOverlap_Object_5and_Object_6B: y_5 + 0.5 w_5 - y_6 + 0.5 w_6
   + 2.260000000894069 C142 <= 2.260000000894069
 NonOverlap_Object_5and_Object_6U: z_5 - 0.5 h_5 - z_6 - 0.5 h_6
   - 2.260000000894069 C143 >= -2.260000000894069
 NonOverlap_Object_5and_Object_6D: z_5 + 0.5 h_5 - z_6 + 0.5 h_6
   + 2.260000000894069 C144 <= 2.260000000894069
 NonOverlap_Object_5and_Object_6: C139 + C140 + C141 + C142 + C143 + C144
   >= 1
 NonOverlap_Object_5and_Object_7R: x_5 - 0.5 l_5 - x_7 - 0.5 l_7
   - 2.260000000894069 C145 >= -2.260000000894069
 NonOverlap_Object_5and_Object_7L: x_5 + 0.5 l_5 - x_7 + 0.5 l_7
   + 2.260000000894069 C146 <= 2.260000000894069
 NonOverlap_Object_5and_Object_7F: y_5 - 0.5 w_5 - y_7 - 0.5 w_7
   - 2.260000000894069 C147 >= -2.260000000894069
 NonOverlap_Object_5and_Object_7B: y_5 + 0.5 w_5 - y_7 + 0.5 w_7
   + 2.260000000894069 C148 <= 2.260000000894069
 NonOverlap_Object_5and_Object_7U: z_5 - 0.5 h_5 - z_7 - 0.5 h_7
   - 2.260000000894069 C149 >= -2.260000000894069
 NonOverlap_Object_5and_Object_7D: z_5 + 0.5 h_5 - z_7 + 0.5 h_7
   + 2.260000000894069 C150 <= 2.260000000894069
 NonOverlap_Object_5and_Object_7: C145 + C146 + C147 + C148 + C149 + C150
   >= 1
 NonOverlap_Object_6and_Object_7R: x_6 - 0.5 l_6 - x_7 - 0.5 l_7
   - 2.260000000894069 C151 >= -2.260000000894069
 NonOverlap_Object_6and_Object_7L: x_6 + 0.5 l_6 - x_7 + 0.5 l_7
   + 2.260000000894069 C152 <= 2.260000000894069
 NonOverlap_Object_6and_Object_7F: y_6 - 0.5 w_6 - y_7 - 0.5 w_7
   - 2.260000000894069 C153 >= -2.260000000894069
 NonOverlap_Object_6and_Object_7B: y_6 + 0.5 w_6 - y_7 + 0.5 w_7
   + 2.260000000894069 C154 <= 2.260000000894069
 NonOverlap_Object_6and_Object_7U: z_6 - 0.5 h_6 - z_7 - 0.5 h_7
   - 2.260000000894069 C155 >= -2.260000000894069
 NonOverlap_Object_6and_Object_7D: z_6 + 0.5 h_6 - z_7 + 0.5 h_7
   + 2.260000000894069 C156 <= 2.260000000894069
 NonOverlap_Object_6and_Object_7: C151 + C152 + C153 + C154 + C155 + C156
   >= 1
 NonOverlap_Object_0and_Obstacle_0L: x_0 + 0.5 l_0 + 2.260000000894069 C157
   <= 2.620000000894069
 NonOverlap_Object_0and_Obstacle_0R: x_0 - 0.5 l_0 - 2.260000000894069 C158
   >= -1.82000000089407
 NonOverlap_Object_0and_Obstacle_0F: y_0 - 0.5 w_0 - 2.260000000894069 C159
   >= -2.180000000894069
 NonOverlap_Object_0and_Obstacle_0: C157 + C158 + C159 >= 1
 NonOverlap_Object_0and_Obstacle_1R: x_0 - 0.5 l_0 - 2.260000000894069 C160
   >= -2.060000000894069
 NonOverlap_Object_0and_Obstacle_1B: y_0 + 0.5 w_0 + 2.260000000894069 C161
   <= 2.760000000894069
 NonOverlap_Object_0and_Obstacle_1D: z_0 + 0.5 h_0 + 2.260000000894069 C162
   <= 2.310000000894069
 NonOverlap_Object_0and_Obstacle_1: C160 + C161 + C162 >= 1
 NonOverlap_Object_0and_Obstacle_2L: x_0 + 0.5 l_0 + 2.260000000894069 C163
   <= 2.950000000894069
 NonOverlap_Object_0and_Obstacle_2F: y_0 - 0.5 w_0 - 2.260000000894069 C164
   >= -1.950000000894069
 NonOverlap_Object_0and_Obstacle_2: C163 + C164 >= 1
 NonOverlap_Object_1and_Obstacle_0L: x_1 + 0.5 l_1 + 2.260000000894069 C165
   <= 2.620000000894069
 NonOverlap_Object_1and_Obstacle_0R: x_1 - 0.5 l_1 - 2.260000000894069 C166
   >= -1.82000000089407
 NonOverlap_Object_1and_Obstacle_0F: y_1 - 0.5 w_1 - 2.260000000894069 C167
   >= -2.180000000894069
 NonOverlap_Object_1and_Obstacle_0: C165 + C166 + C167 >= 1
 NonOverlap_Object_1and_Obstacle_1R: x_1 - 0.5 l_1 - 2.260000000894069 C168
   >= -2.060000000894069
 NonOverlap_Object_1and_Obstacle_1B: y_1 + 0.5 w_1 + 2.260000000894069 C169
   <= 2.760000000894069
 NonOverlap_Object_1and_Obstacle_1D: z_1 + 0.5 h_1 + 2.260000000894069 C170
   <= 2.310000000894069
 NonOverlap_Object_1and_Obstacle_1: C168 + C169 + C170 >= 1
 NonOverlap_Object_1and_Obstacle_2L: x_1 + 0.5 l_1 + 2.260000000894069 C171
   <= 2.950000000894069
 NonOverlap_Object_1and_Obstacle_2F: y_1 - 0.5 w_1 - 2.260000000894069 C172
   >= -1.950000000894069
 NonOverlap_Object_1and_Obstacle_2: C171 + C172 >= 1
 NonOverlap_Object_2and_Obstacle_0L: x_2 + 0.5 l_2 + 2.260000000894069 C173
   <= 2.620000000894069
 NonOverlap_Object_2and_Obstacle_0R: x_2 - 0.5 l_2 - 2.260000000894069 C174
   >= -1.82000000089407
 NonOverlap_Object_2and_Obstacle_0F: y_2 - 0.5 w_2 - 2.260000000894069 C175
   >= -2.180000000894069
 NonOverlap_Object_2and_Obstacle_0: C173 + C174 + C175 >= 1
 NonOverlap_Object_2and_Obstacle_1R: x_2 - 0.5 l_2 - 2.260000000894069 C176
   >= -2.060000000894069
 NonOverlap_Object_2and_Obstacle_1B: y_2 + 0.5 w_2 + 2.260000000894069 C177
   <= 2.760000000894069
 NonOverlap_Object_2and_Obstacle_1D: z_2 + 0.5 h_2 + 2.260000000894069 C178
   <= 2.310000000894069
 NonOverlap_Object_2and_Obstacle_1: C176 + C177 + C178 >= 1
 NonOverlap_Object_2and_Obstacle_2L: x_2 + 0.5 l_2 + 2.260000000894069 C179
   <= 2.950000000894069
 NonOverlap_Object_2and_Obstacle_2F: y_2 - 0.5 w_2 - 2.260000000894069 C180
   >= -1.950000000894069
 NonOverlap_Object_2and_Obstacle_2: C179 + C180 >= 1
 NonOverlap_Object_3and_Obstacle_0L: x_3 + 0.5 l_3 + 2.260000000894069 C181
   <= 2.620000000894069
 NonOverlap_Object_3and_Obstacle_0R: x_3 - 0.5 l_3 - 2.260000000894069 C182
   >= -1.82000000089407
 NonOverlap_Object_3and_Obstacle_0F: y_3 - 0.5 w_3 - 2.260000000894069 C183
   >= -2.180000000894069
 NonOverlap_Object_3and_Obstacle_0: C181 + C182 + C183 >= 1
 NonOverlap_Object_3and_Obstacle_1R: x_3 - 0.5 l_3 - 2.260000000894069 C184
   >= -2.060000000894069
 NonOverlap_Object_3and_Obstacle_1B: y_3 + 0.5 w_3 + 2.260000000894069 C185
   <= 2.760000000894069
 NonOverlap_Object_3and_Obstacle_1D: z_3 + 0.5 h_3 + 2.260000000894069 C186
   <= 2.310000000894069
 NonOverlap_Object_3and_Obstacle_1: C184 + C185 + C186 >= 1
 NonOverlap_Object_3and_Obstacle_2L: x_3 + 0.5 l_3 + 2.260000000894069 C187
   <= 2.950000000894069
 NonOverlap_Object_3and_Obstacle_2F: y_3 - 0.5 w_3 - 2.260000000894069 C188
   >= -1.950000000894069
 NonOverlap_Object_3and_Obstacle_2: C187 + C188 >= 1
 NonOverlap_Object_4and_Obstacle_0L: x_4 + 0.5 l_4 + 2.260000000894069 C189
   <= 2.620000000894069
 NonOverlap_Object_4and_Obstacle_0R: x_4 - 0.5 l_4 - 2.260000000894069 C190
   >= -1.82000000089407
 NonOverlap_Object_4and_Obstacle_0F: y_4 - 0.5 w_4 - 2.260000000894069 C191
   >= -2.180000000894069
 NonOverlap_Object_4and_Obstacle_0: C189 + C190 + C191 >= 1
 NonOverlap_Object_4and_Obstacle_1R: x_4 - 0.5 l_4 - 2.260000000894069 C192
   >= -2.060000000894069
 NonOverlap_Object_4and_Obstacle_1B: y_4 + 0.5 w_4 + 2.260000000894069 C193
   <= 2.760000000894069
 NonOverlap_Object_4and_Obstacle_1D: z_4 + 0.5 h_4 + 2.260000000894069 C194
   <= 2.310000000894069
 NonOverlap_Object_4and_Obstacle_1: C192 + C193 + C194 >= 1
 NonOverlap_Object_4and_Obstacle_2L: x_4 + 0.5 l_4 + 2.260000000894069 C195
   <= 2.950000000894069
 NonOverlap_Object_4and_Obstacle_2F: y_4 - 0.5 w_4 - 2.260000000894069 C196
   >= -1.950000000894069
 NonOverlap_Object_4and_Obstacle_2: C195 + C196 >= 1
 NonOverlap_Object_5and_Obstacle_0L: x_5 + 0.5 l_5 + 2.260000000894069 C197
   <= 2.620000000894069
 NonOverlap_Object_5and_Obstacle_0R: x_5 - 0.5 l_5 - 2.260000000894069 C198
   >= -1.82000000089407
 NonOverlap_Object_5and_Obstacle_0F: y_5 - 0.5 w_5 - 2.260000000894069 C199
   >= -2.180000000894069
 NonOverlap_Object_5and_Obstacle_0: C197 + C198 + C199 >= 1
 NonOverlap_Object_5and_Obstacle_1R: x_5 - 0.5 l_5 - 2.260000000894069 C200
   >= -2.060000000894069
 NonOverlap_Object_5and_Obstacle_1B: y_5 + 0.5 w_5 + 2.260000000894069 C201
   <= 2.760000000894069
 NonOverlap_Object_5and_Obstacle_1D: z_5 + 0.5 h_5 + 2.260000000894069 C202
   <= 2.310000000894069
 NonOverlap_Object_5and_Obstacle_1: C200 + C201 + C202 >= 1
 NonOverlap_Object_5and_Obstacle_2L: x_5 + 0.5 l_5 + 2.260000000894069 C203
   <= 2.950000000894069
 NonOverlap_Object_5and_Obstacle_2F: y_5 - 0.5 w_5 - 2.260000000894069 C204
   >= -1.950000000894069
 NonOverlap_Object_5and_Obstacle_2: C203 + C204 >= 1
 NonOverlap_Object_6and_Obstacle_0L: x_6 + 0.5 l_6 + 2.260000000894069 C205
   <= 2.620000000894069
 NonOverlap_Object_6and_Obstacle_0R: x_6 - 0.5 l_6 - 2.260000000894069 C206
   >= -1.82000000089407
 NonOverlap_Object_6and_Obstacle_0F: y_6 - 0.5 w_6 - 2.260000000894069 C207
   >= -2.180000000894069
 NonOverlap_Object_6and_Obstacle_0: C205 + C206 + C207 >= 1
 NonOverlap_Object_6and_Obstacle_1R: x_6 - 0.5 l_6 - 2.260000000894069 C208
   >= -2.060000000894069
 NonOverlap_Object_6and_Obstacle_1B: y_6 + 0.5 w_6 + 2.260000000894069 C209
   <= 2.760000000894069
 NonOverlap_Object_6and_Obstacle_1D: z_6 + 0.5 h_6 + 2.260000000894069 C210
   <= 2.310000000894069
 NonOverlap_Object_6and_Obstacle_1: C208 + C209 + C210 >= 1
 NonOverlap_Object_6and_Obstacle_2L: x_6 + 0.5 l_6 + 2.260000000894069 C211
   <= 2.950000000894069
 NonOverlap_Object_6and_Obstacle_2F: y_6 - 0.5 w_6 - 2.260000000894069 C212
   >= -1.950000000894069
 NonOverlap_Object_6and_Obstacle_2: C211 + C212 >= 1
 NonOverlap_Object_7and_Obstacle_0L: x_7 + 0.5 l_7 + 2.260000000894069 C213
   <= 2.620000000894069
 NonOverlap_Object_7and_Obstacle_0R: x_7 - 0.5 l_7 - 2.260000000894069 C214
   >= -1.82000000089407
 NonOverlap_Object_7and_Obstacle_0F: y_7 - 0.5 w_7 - 2.260000000894069 C215
   >= -2.180000000894069
 NonOverlap_Object_7and_Obstacle_0: C213 + C214 + C215 >= 1
 NonOverlap_Object_7and_Obstacle_1R: x_7 - 0.5 l_7 - 2.260000000894069 C216
   >= -2.060000000894069
 NonOverlap_Object_7and_Obstacle_1B: y_7 + 0.5 w_7 + 2.260000000894069 C217
   <= 2.760000000894069
 NonOverlap_Object_7and_Obstacle_1D: z_7 + 0.5 h_7 + 2.260000000894069 C218
   <= 2.310000000894069
 NonOverlap_Object_7and_Obstacle_1: C216 + C217 + C218 >= 1
 NonOverlap_Object_7and_Obstacle_2L: x_7 + 0.5 l_7 + 2.260000000894069 C219
   <= 2.950000000894069
 NonOverlap_Object_7and_Obstacle_2F: y_7 - 0.5 w_7 - 2.260000000894069 C220
   >= -1.950000000894069
 NonOverlap_Object_7and_Obstacle_2: C219 + C220 >= 1
 Boundary_Object_0_Right_eq: x_0 + 0.5 l_0 = 0.99
 Boundary_Object_0_Right_ieq: y_0 >= 0.31
 Boundary_Object_0_Right_ieqq: y_0 <= 0.99
 Boundary_Object_3_Front_eq: y_3 + 0.5 w_3 = 0.99
 Boundary_Object_3_Front_ieq: x_3 >= 0.01
 Boundary_Object_3_Front_ieqq: x_3 <= 0.99
 Boundary_Object_4_Left_eq: x_4 - 0.5 l_4 = 0.01
 Boundary_Object_4_Left_ieq: y_4 >= 0.01
 Boundary_Object_4_Left_ieqq: y_4 <= 0.99
 Boundary_Object_6_Left_eq: x_6 - 0.5 l_6 = 0.01
 Boundary_Object_6_Left_ieq: y_6 >= 0.01
 Boundary_Object_6_Left_ieqq: y_6 <= 0.99
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
 C48 C49 C50 C51 C52 C53 C54 C55 C56 C57 C58 C59 C60 C61 C62 C63 C64 C65
 C66 C67 C68 C69 C70 C71 C72 C73 C74 C75 C76 C77 C78 C79 C80 C81 C82 C83
 C84 C85 C86 C87 C88 C89 C90 C91 C92 C93 C94 C95 C96 C97 C98 C99 C100 C101
 C102 C103 C104 C105 C106 C107 C108 C109 C110 C111 C112 C113 C114 C115 C116
 C117 C118 C119 C120 C121 C122 C123 C124 C125 C126 C127 C128 C129 C130 C131
 C132 C133 C134 C135 C136 C137 C138 C139 C140 C141 C142 C143 C144 C145 C146
 C147 C148 C149 C150 C151 C152 C153 C154 C155 C156 C157 C158 C159 C160 C161
 C162 C163 C164 C165 C166 C167 C168 C169 C170 C171 C172 C173 C174 C175 C176
 C177 C178 C179 C180 C181 C182 C183 C184 C185 C186 C187 C188 C189 C190 C191
 C192 C193 C194 C195 C196 C197 C198 C199 C200 C201 C202 C203 C204 C205 C206
 C207 C208 C209 C210 C211 C212 C213 C214 C215 C216 C217 C218 C219 C220
End
