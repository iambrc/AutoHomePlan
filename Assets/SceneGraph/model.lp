\ LP format - for model browsing. Use MPS format to capture full model detail.
\ Signature: 0xadbbc0be19132874
Minimize
  - 0.4164931274834169 x_0 - 0.6247396912251254 l_0 - 0.4164931274834169 w_0
   - 2.222222222222223 h_0 - 0.2082465637417084 l_1
   - 0.2082465637417084 w_1 - 1.777777777777778 h_1
   - 0.2082465637417084 l_2 - 0.2082465637417084 w_2
   - 1.777777777777778 h_2 + 0.4164931274834169 x_3
   - 0.4164931274834169 y_3 - 0.6247396912251254 l_3
   - 0.1041232818708542 w_3 - 4.444444444444446 h_3
   + 0.2290712201158793 x_4 + 0.4164931274834169 y_4
   - 0.3019575174254773 l_4 - 0.3123698456125626 w_4
   - 2.666666666666667 h_4 - 0.2290712201158793 x_5
   - 0.1561849228062813 l_5 - 0.1665972509933668 w_5
   - 1.777777777777778 h_5 - 1.041232818708542 x_6 - 1.041232818708542 y_6
   - 0.0832986254966834 l_6 - 0.0832986254966834 w_6
   - 5.555555555555556 z_6 - 2.222222222222223 h_6 - 0.0416493127483417 l_7
   - 0.0416493127483417 w_7 - 0.4444444444444445 h_7
   + 3.399238742057688 Constant + [ 2.082465637417084 x_0 ^2
   - 2.082465637417084 x_0 * l_0 - 4.164931274834169 x_0 * x_3
   - 2.082465637417084 x_0 * l_3 + 4.164931274834169 y_0 ^2
   - 4.164931274834169 y_0 * y_2 - 2.082465637417084 y_0 * w_2
   - 4.164931274834169 y_0 * y_1 + 2.082465637417084 y_0 * w_1
   - 1.041232818708542 l_0 * w_0 + 2.603082046771355 l_0 ^2
   + 2.082465637417084 l_0 * x_3 + 1.041232818708542 l_0 * l_3
   + 3.123698456125626 w_0 ^2 + 2.082465637417084 w_0 * y_2
   + 1.041232818708542 w_0 * w_2 - 2.082465637417084 w_0 * y_1
   + 1.041232818708542 w_0 * w_1 + 22.22222222222222 h_0 ^2
   + 2.082465637417084 y_1 ^2 - 2.082465637417084 y_1 * w_1
   - 1.041232818708542 l_1 * w_1 + 2.082465637417084 l_1 ^2
   + 2.603082046771355 w_1 ^2 + 22.22222222222222 h_1 ^2
   + 2.082465637417084 y_2 ^2 + 2.082465637417084 y_2 * w_2
   - 1.041232818708542 l_2 * w_2 + 2.082465637417084 l_2 ^2
   + 2.603082046771355 w_2 ^2 + 22.22222222222222 h_2 ^2
   + 2.082465637417084 x_3 ^2 + 2.082465637417084 x_3 * l_3
   + 2.082465637417084 y_3 ^2 - 2.082465637417084 y_3 * w_3
   - 4.164931274834169 y_3 * y_4 - 2.082465637417084 y_3 * w_4
   - 1.041232818708542 l_3 * w_3 + 2.603082046771355 l_3 ^2
   + 2.603082046771355 w_3 ^2 + 2.082465637417084 w_3 * y_4
   + 1.041232818708542 w_3 * w_4 + 22.22222222222222 h_3 ^2
   - 8.329862549668338 x_4 * x_5 + 2.082465637417084 x_4 * l_5
   + 6.247396912251253 x_4 ^2 + 2.082465637417084 x_4 * l_4
   - 4.164931274834169 x_4 * x_7 + 6.247396912251253 y_4 ^2
   + 2.082465637417084 y_4 * w_4 - 4.164931274834169 y_4 * y_5
   - 4.164931274834169 y_4 * y_7 - 1.041232818708542 l_4 * w_4
   + 2.603082046771355 l_4 ^2 - 2.082465637417084 l_4 * x_5
   + 1.041232818708542 l_4 * l_5 + 2.603082046771355 w_4 ^2
   + 22.22222222222222 h_4 ^2 + 4.164931274834169 x_5 ^2
   - 2.082465637417084 x_5 * l_5 + 2.082465637417084 y_5 ^2
   - 1.041232818708542 l_5 * w_5 + 2.603082046771355 l_5 ^2
   + 2.082465637417084 w_5 ^2 + 22.22222222222222 h_5 ^2
   + 2.082465637417084 x_6 ^2 + 2.082465637417084 y_6 ^2
   - 1.041232818708542 l_6 * w_6 + 2.082465637417084 l_6 ^2
   + 2.082465637417084 w_6 ^2 + 22.22222222222222 z_6 ^2
   + 22.22222222222222 h_6 ^2 + 2.082465637417084 x_7 ^2
   + 2.082465637417084 y_7 ^2 + 2.082465637417084 l_7 ^2
   + 2.082465637417084 w_7 ^2 + 22.22222222222222 h_7 ^2 ] / 2 
Subject To
 Inside_Object_0_x_left: x_0 - 0.5 l_0 >= 0.00999999977648258
 Inside_Object_0_x_right: x_0 + 0.5 l_0 <= 0.9900000002235174
 Inside_Object_0_y_back: y_0 - 0.5 w_0 >= 0.00999999977648258
 Inside_Object_0_y_front: y_0 + 0.5 w_0 <= 0.9900000002235174
 Inside_Object_0_z_bottom: z_0 - 0.5 h_0 >= 0
 Inside_Object_0_z_top: z_0 + 0.5 h_0 <= 0.3
 Inside_Object_1_x_left: x_1 - 0.5 l_1 >= 0.00999999977648258
 Inside_Object_1_x_right: x_1 + 0.5 l_1 <= 0.9900000002235174
 Inside_Object_1_y_back: y_1 - 0.5 w_1 >= 0.00999999977648258
 Inside_Object_1_y_front: y_1 + 0.5 w_1 <= 0.9900000002235174
 Inside_Object_1_z_bottom: z_1 - 0.5 h_1 >= 0
 Inside_Object_1_z_top: z_1 + 0.5 h_1 <= 0.3
 Inside_Object_2_x_left: x_2 - 0.5 l_2 >= 0.00999999977648258
 Inside_Object_2_x_right: x_2 + 0.5 l_2 <= 0.9900000002235174
 Inside_Object_2_y_back: y_2 - 0.5 w_2 >= 0.00999999977648258
 Inside_Object_2_y_front: y_2 + 0.5 w_2 <= 0.9900000002235174
 Inside_Object_2_z_bottom: z_2 - 0.5 h_2 >= 0
 Inside_Object_2_z_top: z_2 + 0.5 h_2 <= 0.3
 Inside_Object_3_x_left: x_3 - 0.5 l_3 >= 0.00999999977648258
 Inside_Object_3_x_right: x_3 + 0.5 l_3 <= 0.9900000002235174
 Inside_Object_3_y_back: y_3 - 0.5 w_3 >= 0.00999999977648258
 Inside_Object_3_y_front: y_3 + 0.5 w_3 <= 0.9900000002235174
 Inside_Object_3_z_bottom: z_3 - 0.5 h_3 >= 0
 Inside_Object_3_z_top: z_3 + 0.5 h_3 <= 0.3
 Inside_Object_4_x_left: x_4 - 0.5 l_4 >= 0.00999999977648258
 Inside_Object_4_x_right: x_4 + 0.5 l_4 <= 0.9900000002235174
 Inside_Object_4_y_back: y_4 - 0.5 w_4 >= 0.00999999977648258
 Inside_Object_4_y_front: y_4 + 0.5 w_4 <= 0.9900000002235174
 Inside_Object_4_z_bottom: z_4 - 0.5 h_4 >= 0
 Inside_Object_4_z_top: z_4 + 0.5 h_4 <= 0.3
 Inside_Object_5_x_left: x_5 - 0.5 l_5 >= 0.00999999977648258
 Inside_Object_5_x_right: x_5 + 0.5 l_5 <= 0.9900000002235174
 Inside_Object_5_y_back: y_5 - 0.5 w_5 >= 0.00999999977648258
 Inside_Object_5_y_front: y_5 + 0.5 w_5 <= 0.9900000002235174
 Inside_Object_5_z_bottom: z_5 - 0.5 h_5 >= 0
 Inside_Object_5_z_top: z_5 + 0.5 h_5 <= 0.3
 Inside_Object_6_x_left: x_6 - 0.5 l_6 >= 0.00999999977648258
 Inside_Object_6_x_right: x_6 + 0.5 l_6 <= 0.9900000002235174
 Inside_Object_6_y_back: y_6 - 0.5 w_6 >= 0.00999999977648258
 Inside_Object_6_y_front: y_6 + 0.5 w_6 <= 0.9900000002235174
 Inside_Object_6_z_bottom: z_6 - 0.5 h_6 >= 0
 Inside_Object_6_z_top: z_6 + 0.5 h_6 <= 0.3
 Inside_Object_7_x_left: x_7 - 0.5 l_7 >= 0.00999999977648258
 Inside_Object_7_x_right: x_7 + 0.5 l_7 <= 0.9900000002235174
 Inside_Object_7_y_back: y_7 - 0.5 w_7 >= 0.00999999977648258
 Inside_Object_7_y_front: y_7 + 0.5 w_7 <= 0.9900000002235174
 Inside_Object_7_z_bottom: z_7 - 0.5 h_7 >= 0
 Inside_Object_7_z_top: z_7 + 0.5 h_7 <= 0.3
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
   + 2.260000000894069 C49 <= 2.260000000894069
 Object_5_CloseBy_Object_4ieqb: - x_4 + 0.5 l_4 + x_5 + 0.5 l_5
   - 2.260000000894069 C48 >= -2.260000000894069
 Object_5_CloseBy_Object_4ieqc: - y_4 - 0.5 w_4 + y_5 - 0.5 w_5
   + 2.260000000894069 C50 <= 2.260000000894069
 Object_5_CloseBy_Object_4ieqd: - y_4 + 0.5 w_4 + y_5 + 0.5 w_5
   - 2.260000000894069 C51 >= -2.260000000894069
 Object_5_CloseBy_Object_4ieqe: C48 + C49 + C50 + C51 <= 1
 Object_7_Above_Object_4: - z_4 - 0.5 h_4 + z_7 - 0.5 h_7 = 0
 NonOverlap_Object_0and_Object_4R: x_0 - 0.5 l_0 - x_4 - 0.5 l_4
   - 2.260000000894069 C52 >= -2.260000000894069
 NonOverlap_Object_0and_Object_4L: x_0 + 0.5 l_0 - x_4 + 0.5 l_4
   + 2.260000000894069 C53 <= 2.260000000894069
 NonOverlap_Object_0and_Object_4F: y_0 - 0.5 w_0 - y_4 - 0.5 w_4
   - 2.260000000894069 C54 >= -2.260000000894069
 NonOverlap_Object_0and_Object_4B: y_0 + 0.5 w_0 - y_4 + 0.5 w_4
   + 2.260000000894069 C55 <= 2.260000000894069
 NonOverlap_Object_0and_Object_4U: z_0 - 0.5 h_0 - z_4 - 0.5 h_4
   - 2.260000000894069 C56 >= -2.260000000894069
 NonOverlap_Object_0and_Object_4D: z_0 + 0.5 h_0 - z_4 + 0.5 h_4
   + 2.260000000894069 C57 <= 2.260000000894069
 NonOverlap_Object_0and_Object_4: C52 + C53 + C54 + C55 + C56 + C57 >= 1
 NonOverlap_Object_0and_Object_5R: x_0 - 0.5 l_0 - x_5 - 0.5 l_5
   - 2.260000000894069 C58 >= -2.260000000894069
 NonOverlap_Object_0and_Object_5L: x_0 + 0.5 l_0 - x_5 + 0.5 l_5
   + 2.260000000894069 C59 <= 2.260000000894069
 NonOverlap_Object_0and_Object_5F: y_0 - 0.5 w_0 - y_5 - 0.5 w_5
   - 2.260000000894069 C60 >= -2.260000000894069
 NonOverlap_Object_0and_Object_5B: y_0 + 0.5 w_0 - y_5 + 0.5 w_5
   + 2.260000000894069 C61 <= 2.260000000894069
 NonOverlap_Object_0and_Object_5U: z_0 - 0.5 h_0 - z_5 - 0.5 h_5
   - 2.260000000894069 C62 >= -2.260000000894069
 NonOverlap_Object_0and_Object_5D: z_0 + 0.5 h_0 - z_5 + 0.5 h_5
   + 2.260000000894069 C63 <= 2.260000000894069
 NonOverlap_Object_0and_Object_5: C58 + C59 + C60 + C61 + C62 + C63 >= 1
 NonOverlap_Object_0and_Object_6R: x_0 - 0.5 l_0 - x_6 - 0.5 l_6
   - 2.260000000894069 C64 >= -2.260000000894069
 NonOverlap_Object_0and_Object_6L: x_0 + 0.5 l_0 - x_6 + 0.5 l_6
   + 2.260000000894069 C65 <= 2.260000000894069
 NonOverlap_Object_0and_Object_6F: y_0 - 0.5 w_0 - y_6 - 0.5 w_6
   - 2.260000000894069 C66 >= -2.260000000894069
 NonOverlap_Object_0and_Object_6B: y_0 + 0.5 w_0 - y_6 + 0.5 w_6
   + 2.260000000894069 C67 <= 2.260000000894069
 NonOverlap_Object_0and_Object_6U: z_0 - 0.5 h_0 - z_6 - 0.5 h_6
   - 2.260000000894069 C68 >= -2.260000000894069
 NonOverlap_Object_0and_Object_6D: z_0 + 0.5 h_0 - z_6 + 0.5 h_6
   + 2.260000000894069 C69 <= 2.260000000894069
 NonOverlap_Object_0and_Object_6: C64 + C65 + C66 + C67 + C68 + C69 >= 1
 NonOverlap_Object_0and_Object_7R: x_0 - 0.5 l_0 - x_7 - 0.5 l_7
   - 2.260000000894069 C70 >= -2.260000000894069
 NonOverlap_Object_0and_Object_7L: x_0 + 0.5 l_0 - x_7 + 0.5 l_7
   + 2.260000000894069 C71 <= 2.260000000894069
 NonOverlap_Object_0and_Object_7F: y_0 - 0.5 w_0 - y_7 - 0.5 w_7
   - 2.260000000894069 C72 >= -2.260000000894069
 NonOverlap_Object_0and_Object_7B: y_0 + 0.5 w_0 - y_7 + 0.5 w_7
   + 2.260000000894069 C73 <= 2.260000000894069
 NonOverlap_Object_0and_Object_7U: z_0 - 0.5 h_0 - z_7 - 0.5 h_7
   - 2.260000000894069 C74 >= -2.260000000894069
 NonOverlap_Object_0and_Object_7D: z_0 + 0.5 h_0 - z_7 + 0.5 h_7
   + 2.260000000894069 C75 <= 2.260000000894069
 NonOverlap_Object_0and_Object_7: C70 + C71 + C72 + C73 + C74 + C75 >= 1
 NonOverlap_Object_1and_Object_3R: x_1 - 0.5 l_1 - x_3 - 0.5 l_3
   - 2.260000000894069 C76 >= -2.260000000894069
 NonOverlap_Object_1and_Object_3L: x_1 + 0.5 l_1 - x_3 + 0.5 l_3
   + 2.260000000894069 C77 <= 2.260000000894069
 NonOverlap_Object_1and_Object_3F: y_1 - 0.5 w_1 - y_3 - 0.5 w_3
   - 2.260000000894069 C78 >= -2.260000000894069
 NonOverlap_Object_1and_Object_3B: y_1 + 0.5 w_1 - y_3 + 0.5 w_3
   + 2.260000000894069 C79 <= 2.260000000894069
 NonOverlap_Object_1and_Object_3U: z_1 - 0.5 h_1 - z_3 - 0.5 h_3
   - 2.260000000894069 C80 >= -2.260000000894069
 NonOverlap_Object_1and_Object_3D: z_1 + 0.5 h_1 - z_3 + 0.5 h_3
   + 2.260000000894069 C81 <= 2.260000000894069
 NonOverlap_Object_1and_Object_3: C76 + C77 + C78 + C79 + C80 + C81 >= 1
 NonOverlap_Object_1and_Object_4R: x_1 - 0.5 l_1 - x_4 - 0.5 l_4
   - 2.260000000894069 C82 >= -2.260000000894069
 NonOverlap_Object_1and_Object_4L: x_1 + 0.5 l_1 - x_4 + 0.5 l_4
   + 2.260000000894069 C83 <= 2.260000000894069
 NonOverlap_Object_1and_Object_4F: y_1 - 0.5 w_1 - y_4 - 0.5 w_4
   - 2.260000000894069 C84 >= -2.260000000894069
 NonOverlap_Object_1and_Object_4B: y_1 + 0.5 w_1 - y_4 + 0.5 w_4
   + 2.260000000894069 C85 <= 2.260000000894069
 NonOverlap_Object_1and_Object_4U: z_1 - 0.5 h_1 - z_4 - 0.5 h_4
   - 2.260000000894069 C86 >= -2.260000000894069
 NonOverlap_Object_1and_Object_4D: z_1 + 0.5 h_1 - z_4 + 0.5 h_4
   + 2.260000000894069 C87 <= 2.260000000894069
 NonOverlap_Object_1and_Object_4: C82 + C83 + C84 + C85 + C86 + C87 >= 1
 NonOverlap_Object_1and_Object_5R: x_1 - 0.5 l_1 - x_5 - 0.5 l_5
   - 2.260000000894069 C88 >= -2.260000000894069
 NonOverlap_Object_1and_Object_5L: x_1 + 0.5 l_1 - x_5 + 0.5 l_5
   + 2.260000000894069 C89 <= 2.260000000894069
 NonOverlap_Object_1and_Object_5F: y_1 - 0.5 w_1 - y_5 - 0.5 w_5
   - 2.260000000894069 C90 >= -2.260000000894069
 NonOverlap_Object_1and_Object_5B: y_1 + 0.5 w_1 - y_5 + 0.5 w_5
   + 2.260000000894069 C91 <= 2.260000000894069
 NonOverlap_Object_1and_Object_5U: z_1 - 0.5 h_1 - z_5 - 0.5 h_5
   - 2.260000000894069 C92 >= -2.260000000894069
 NonOverlap_Object_1and_Object_5D: z_1 + 0.5 h_1 - z_5 + 0.5 h_5
   + 2.260000000894069 C93 <= 2.260000000894069
 NonOverlap_Object_1and_Object_5: C88 + C89 + C90 + C91 + C92 + C93 >= 1
 NonOverlap_Object_1and_Object_6R: x_1 - 0.5 l_1 - x_6 - 0.5 l_6
   - 2.260000000894069 C94 >= -2.260000000894069
 NonOverlap_Object_1and_Object_6L: x_1 + 0.5 l_1 - x_6 + 0.5 l_6
   + 2.260000000894069 C95 <= 2.260000000894069
 NonOverlap_Object_1and_Object_6F: y_1 - 0.5 w_1 - y_6 - 0.5 w_6
   - 2.260000000894069 C96 >= -2.260000000894069
 NonOverlap_Object_1and_Object_6B: y_1 + 0.5 w_1 - y_6 + 0.5 w_6
   + 2.260000000894069 C97 <= 2.260000000894069
 NonOverlap_Object_1and_Object_6U: z_1 - 0.5 h_1 - z_6 - 0.5 h_6
   - 2.260000000894069 C98 >= -2.260000000894069
 NonOverlap_Object_1and_Object_6D: z_1 + 0.5 h_1 - z_6 + 0.5 h_6
   + 2.260000000894069 C99 <= 2.260000000894069
 NonOverlap_Object_1and_Object_6: C94 + C95 + C96 + C97 + C98 + C99 >= 1
 NonOverlap_Object_1and_Object_7R: x_1 - 0.5 l_1 - x_7 - 0.5 l_7
   - 2.260000000894069 C100 >= -2.260000000894069
 NonOverlap_Object_1and_Object_7L: x_1 + 0.5 l_1 - x_7 + 0.5 l_7
   + 2.260000000894069 C101 <= 2.260000000894069
 NonOverlap_Object_1and_Object_7F: y_1 - 0.5 w_1 - y_7 - 0.5 w_7
   - 2.260000000894069 C102 >= -2.260000000894069
 NonOverlap_Object_1and_Object_7B: y_1 + 0.5 w_1 - y_7 + 0.5 w_7
   + 2.260000000894069 C103 <= 2.260000000894069
 NonOverlap_Object_1and_Object_7U: z_1 - 0.5 h_1 - z_7 - 0.5 h_7
   - 2.260000000894069 C104 >= -2.260000000894069
 NonOverlap_Object_1and_Object_7D: z_1 + 0.5 h_1 - z_7 + 0.5 h_7
   + 2.260000000894069 C105 <= 2.260000000894069
 NonOverlap_Object_1and_Object_7: C100 + C101 + C102 + C103 + C104 + C105
   >= 1
 NonOverlap_Object_2and_Object_3R: x_2 - 0.5 l_2 - x_3 - 0.5 l_3
   - 2.260000000894069 C106 >= -2.260000000894069
 NonOverlap_Object_2and_Object_3L: x_2 + 0.5 l_2 - x_3 + 0.5 l_3
   + 2.260000000894069 C107 <= 2.260000000894069
 NonOverlap_Object_2and_Object_3F: y_2 - 0.5 w_2 - y_3 - 0.5 w_3
   - 2.260000000894069 C108 >= -2.260000000894069
 NonOverlap_Object_2and_Object_3B: y_2 + 0.5 w_2 - y_3 + 0.5 w_3
   + 2.260000000894069 C109 <= 2.260000000894069
 NonOverlap_Object_2and_Object_3U: z_2 - 0.5 h_2 - z_3 - 0.5 h_3
   - 2.260000000894069 C110 >= -2.260000000894069
 NonOverlap_Object_2and_Object_3D: z_2 + 0.5 h_2 - z_3 + 0.5 h_3
   + 2.260000000894069 C111 <= 2.260000000894069
 NonOverlap_Object_2and_Object_3: C106 + C107 + C108 + C109 + C110 + C111
   >= 1
 NonOverlap_Object_2and_Object_4R: x_2 - 0.5 l_2 - x_4 - 0.5 l_4
   - 2.260000000894069 C112 >= -2.260000000894069
 NonOverlap_Object_2and_Object_4L: x_2 + 0.5 l_2 - x_4 + 0.5 l_4
   + 2.260000000894069 C113 <= 2.260000000894069
 NonOverlap_Object_2and_Object_4F: y_2 - 0.5 w_2 - y_4 - 0.5 w_4
   - 2.260000000894069 C114 >= -2.260000000894069
 NonOverlap_Object_2and_Object_4B: y_2 + 0.5 w_2 - y_4 + 0.5 w_4
   + 2.260000000894069 C115 <= 2.260000000894069
 NonOverlap_Object_2and_Object_4U: z_2 - 0.5 h_2 - z_4 - 0.5 h_4
   - 2.260000000894069 C116 >= -2.260000000894069
 NonOverlap_Object_2and_Object_4D: z_2 + 0.5 h_2 - z_4 + 0.5 h_4
   + 2.260000000894069 C117 <= 2.260000000894069
 NonOverlap_Object_2and_Object_4: C112 + C113 + C114 + C115 + C116 + C117
   >= 1
 NonOverlap_Object_2and_Object_5R: x_2 - 0.5 l_2 - x_5 - 0.5 l_5
   - 2.260000000894069 C118 >= -2.260000000894069
 NonOverlap_Object_2and_Object_5L: x_2 + 0.5 l_2 - x_5 + 0.5 l_5
   + 2.260000000894069 C119 <= 2.260000000894069
 NonOverlap_Object_2and_Object_5F: y_2 - 0.5 w_2 - y_5 - 0.5 w_5
   - 2.260000000894069 C120 >= -2.260000000894069
 NonOverlap_Object_2and_Object_5B: y_2 + 0.5 w_2 - y_5 + 0.5 w_5
   + 2.260000000894069 C121 <= 2.260000000894069
 NonOverlap_Object_2and_Object_5U: z_2 - 0.5 h_2 - z_5 - 0.5 h_5
   - 2.260000000894069 C122 >= -2.260000000894069
 NonOverlap_Object_2and_Object_5D: z_2 + 0.5 h_2 - z_5 + 0.5 h_5
   + 2.260000000894069 C123 <= 2.260000000894069
 NonOverlap_Object_2and_Object_5: C118 + C119 + C120 + C121 + C122 + C123
   >= 1
 NonOverlap_Object_2and_Object_6R: x_2 - 0.5 l_2 - x_6 - 0.5 l_6
   - 2.260000000894069 C124 >= -2.260000000894069
 NonOverlap_Object_2and_Object_6L: x_2 + 0.5 l_2 - x_6 + 0.5 l_6
   + 2.260000000894069 C125 <= 2.260000000894069
 NonOverlap_Object_2and_Object_6F: y_2 - 0.5 w_2 - y_6 - 0.5 w_6
   - 2.260000000894069 C126 >= -2.260000000894069
 NonOverlap_Object_2and_Object_6B: y_2 + 0.5 w_2 - y_6 + 0.5 w_6
   + 2.260000000894069 C127 <= 2.260000000894069
 NonOverlap_Object_2and_Object_6U: z_2 - 0.5 h_2 - z_6 - 0.5 h_6
   - 2.260000000894069 C128 >= -2.260000000894069
 NonOverlap_Object_2and_Object_6D: z_2 + 0.5 h_2 - z_6 + 0.5 h_6
   + 2.260000000894069 C129 <= 2.260000000894069
 NonOverlap_Object_2and_Object_6: C124 + C125 + C126 + C127 + C128 + C129
   >= 1
 NonOverlap_Object_2and_Object_7R: x_2 - 0.5 l_2 - x_7 - 0.5 l_7
   - 2.260000000894069 C130 >= -2.260000000894069
 NonOverlap_Object_2and_Object_7L: x_2 + 0.5 l_2 - x_7 + 0.5 l_7
   + 2.260000000894069 C131 <= 2.260000000894069
 NonOverlap_Object_2and_Object_7F: y_2 - 0.5 w_2 - y_7 - 0.5 w_7
   - 2.260000000894069 C132 >= -2.260000000894069
 NonOverlap_Object_2and_Object_7B: y_2 + 0.5 w_2 - y_7 + 0.5 w_7
   + 2.260000000894069 C133 <= 2.260000000894069
 NonOverlap_Object_2and_Object_7U: z_2 - 0.5 h_2 - z_7 - 0.5 h_7
   - 2.260000000894069 C134 >= -2.260000000894069
 NonOverlap_Object_2and_Object_7D: z_2 + 0.5 h_2 - z_7 + 0.5 h_7
   + 2.260000000894069 C135 <= 2.260000000894069
 NonOverlap_Object_2and_Object_7: C130 + C131 + C132 + C133 + C134 + C135
   >= 1
 NonOverlap_Object_3and_Object_5R: x_3 - 0.5 l_3 - x_5 - 0.5 l_5
   - 2.260000000894069 C136 >= -2.260000000894069
 NonOverlap_Object_3and_Object_5L: x_3 + 0.5 l_3 - x_5 + 0.5 l_5
   + 2.260000000894069 C137 <= 2.260000000894069
 NonOverlap_Object_3and_Object_5F: y_3 - 0.5 w_3 - y_5 - 0.5 w_5
   - 2.260000000894069 C138 >= -2.260000000894069
 NonOverlap_Object_3and_Object_5B: y_3 + 0.5 w_3 - y_5 + 0.5 w_5
   + 2.260000000894069 C139 <= 2.260000000894069
 NonOverlap_Object_3and_Object_5U: z_3 - 0.5 h_3 - z_5 - 0.5 h_5
   - 2.260000000894069 C140 >= -2.260000000894069
 NonOverlap_Object_3and_Object_5D: z_3 + 0.5 h_3 - z_5 + 0.5 h_5
   + 2.260000000894069 C141 <= 2.260000000894069
 NonOverlap_Object_3and_Object_5: C136 + C137 + C138 + C139 + C140 + C141
   >= 1
 NonOverlap_Object_3and_Object_6R: x_3 - 0.5 l_3 - x_6 - 0.5 l_6
   - 2.260000000894069 C142 >= -2.260000000894069
 NonOverlap_Object_3and_Object_6L: x_3 + 0.5 l_3 - x_6 + 0.5 l_6
   + 2.260000000894069 C143 <= 2.260000000894069
 NonOverlap_Object_3and_Object_6F: y_3 - 0.5 w_3 - y_6 - 0.5 w_6
   - 2.260000000894069 C144 >= -2.260000000894069
 NonOverlap_Object_3and_Object_6B: y_3 + 0.5 w_3 - y_6 + 0.5 w_6
   + 2.260000000894069 C145 <= 2.260000000894069
 NonOverlap_Object_3and_Object_6U: z_3 - 0.5 h_3 - z_6 - 0.5 h_6
   - 2.260000000894069 C146 >= -2.260000000894069
 NonOverlap_Object_3and_Object_6D: z_3 + 0.5 h_3 - z_6 + 0.5 h_6
   + 2.260000000894069 C147 <= 2.260000000894069
 NonOverlap_Object_3and_Object_6: C142 + C143 + C144 + C145 + C146 + C147
   >= 1
 NonOverlap_Object_3and_Object_7R: x_3 - 0.5 l_3 - x_7 - 0.5 l_7
   - 2.260000000894069 C148 >= -2.260000000894069
 NonOverlap_Object_3and_Object_7L: x_3 + 0.5 l_3 - x_7 + 0.5 l_7
   + 2.260000000894069 C149 <= 2.260000000894069
 NonOverlap_Object_3and_Object_7F: y_3 - 0.5 w_3 - y_7 - 0.5 w_7
   - 2.260000000894069 C150 >= -2.260000000894069
 NonOverlap_Object_3and_Object_7B: y_3 + 0.5 w_3 - y_7 + 0.5 w_7
   + 2.260000000894069 C151 <= 2.260000000894069
 NonOverlap_Object_3and_Object_7U: z_3 - 0.5 h_3 - z_7 - 0.5 h_7
   - 2.260000000894069 C152 >= -2.260000000894069
 NonOverlap_Object_3and_Object_7D: z_3 + 0.5 h_3 - z_7 + 0.5 h_7
   + 2.260000000894069 C153 <= 2.260000000894069
 NonOverlap_Object_3and_Object_7: C148 + C149 + C150 + C151 + C152 + C153
   >= 1
 NonOverlap_Object_4and_Object_6R: x_4 - 0.5 l_4 - x_6 - 0.5 l_6
   - 2.260000000894069 C154 >= -2.260000000894069
 NonOverlap_Object_4and_Object_6L: x_4 + 0.5 l_4 - x_6 + 0.5 l_6
   + 2.260000000894069 C155 <= 2.260000000894069
 NonOverlap_Object_4and_Object_6F: y_4 - 0.5 w_4 - y_6 - 0.5 w_6
   - 2.260000000894069 C156 >= -2.260000000894069
 NonOverlap_Object_4and_Object_6B: y_4 + 0.5 w_4 - y_6 + 0.5 w_6
   + 2.260000000894069 C157 <= 2.260000000894069
 NonOverlap_Object_4and_Object_6U: z_4 - 0.5 h_4 - z_6 - 0.5 h_6
   - 2.260000000894069 C158 >= -2.260000000894069
 NonOverlap_Object_4and_Object_6D: z_4 + 0.5 h_4 - z_6 + 0.5 h_6
   + 2.260000000894069 C159 <= 2.260000000894069
 NonOverlap_Object_4and_Object_6: C154 + C155 + C156 + C157 + C158 + C159
   >= 1
 NonOverlap_Object_5and_Object_6R: x_5 - 0.5 l_5 - x_6 - 0.5 l_6
   - 2.260000000894069 C160 >= -2.260000000894069
 NonOverlap_Object_5and_Object_6L: x_5 + 0.5 l_5 - x_6 + 0.5 l_6
   + 2.260000000894069 C161 <= 2.260000000894069
 NonOverlap_Object_5and_Object_6F: y_5 - 0.5 w_5 - y_6 - 0.5 w_6
   - 2.260000000894069 C162 >= -2.260000000894069
 NonOverlap_Object_5and_Object_6B: y_5 + 0.5 w_5 - y_6 + 0.5 w_6
   + 2.260000000894069 C163 <= 2.260000000894069
 NonOverlap_Object_5and_Object_6U: z_5 - 0.5 h_5 - z_6 - 0.5 h_6
   - 2.260000000894069 C164 >= -2.260000000894069
 NonOverlap_Object_5and_Object_6D: z_5 + 0.5 h_5 - z_6 + 0.5 h_6
   + 2.260000000894069 C165 <= 2.260000000894069
 NonOverlap_Object_5and_Object_6: C160 + C161 + C162 + C163 + C164 + C165
   >= 1
 NonOverlap_Object_5and_Object_7R: x_5 - 0.5 l_5 - x_7 - 0.5 l_7
   - 2.260000000894069 C166 >= -2.260000000894069
 NonOverlap_Object_5and_Object_7L: x_5 + 0.5 l_5 - x_7 + 0.5 l_7
   + 2.260000000894069 C167 <= 2.260000000894069
 NonOverlap_Object_5and_Object_7F: y_5 - 0.5 w_5 - y_7 - 0.5 w_7
   - 2.260000000894069 C168 >= -2.260000000894069
 NonOverlap_Object_5and_Object_7B: y_5 + 0.5 w_5 - y_7 + 0.5 w_7
   + 2.260000000894069 C169 <= 2.260000000894069
 NonOverlap_Object_5and_Object_7U: z_5 - 0.5 h_5 - z_7 - 0.5 h_7
   - 2.260000000894069 C170 >= -2.260000000894069
 NonOverlap_Object_5and_Object_7D: z_5 + 0.5 h_5 - z_7 + 0.5 h_7
   + 2.260000000894069 C171 <= 2.260000000894069
 NonOverlap_Object_5and_Object_7: C166 + C167 + C168 + C169 + C170 + C171
   >= 1
 NonOverlap_Object_6and_Object_7R: x_6 - 0.5 l_6 - x_7 - 0.5 l_7
   - 2.260000000894069 C172 >= -2.260000000894069
 NonOverlap_Object_6and_Object_7L: x_6 + 0.5 l_6 - x_7 + 0.5 l_7
   + 2.260000000894069 C173 <= 2.260000000894069
 NonOverlap_Object_6and_Object_7F: y_6 - 0.5 w_6 - y_7 - 0.5 w_7
   - 2.260000000894069 C174 >= -2.260000000894069
 NonOverlap_Object_6and_Object_7B: y_6 + 0.5 w_6 - y_7 + 0.5 w_7
   + 2.260000000894069 C175 <= 2.260000000894069
 NonOverlap_Object_6and_Object_7U: z_6 - 0.5 h_6 - z_7 - 0.5 h_7
   - 2.260000000894069 C176 >= -2.260000000894069
 NonOverlap_Object_6and_Object_7D: z_6 + 0.5 h_6 - z_7 + 0.5 h_7
   + 2.260000000894069 C177 <= 2.260000000894069
 NonOverlap_Object_6and_Object_7: C172 + C173 + C174 + C175 + C176 + C177
   >= 1
 NonOverlap_Object_0and_Obstacle_0L: x_0 + 0.5 l_0 + 2.260000000894069 C178
   <= 2.620000000894069
 NonOverlap_Object_0and_Obstacle_0R: x_0 - 0.5 l_0 - 2.260000000894069 C179
   >= -1.82000000089407
 NonOverlap_Object_0and_Obstacle_0B: y_0 + 0.5 w_0 + 2.260000000894069 C180
   <= 2.260000000894069
 NonOverlap_Object_0and_Obstacle_0F: y_0 - 0.5 w_0 - 2.260000000894069 C181
   >= -2.180000000894069
 NonOverlap_Object_0and_Obstacle_0D: z_0 + 0.5 h_0 + 2.260000000894069 C182
   <= 2.260000000894069
 NonOverlap_Object_0and_Obstacle_0U: z_0 - 0.5 h_0 - 2.260000000894069 C183
   >= -2.020000000894069
 NonOverlap_Object_0and_Obstacle_0: C178 + C179 + C180 + C181 + C182 + C183
   >= 1
 NonOverlap_Object_0and_Obstacle_1L: x_0 + 0.5 l_0 + 2.260000000894069 C184
   <= 2.260000000894069
 NonOverlap_Object_0and_Obstacle_1R: x_0 - 0.5 l_0 - 2.260000000894069 C185
   >= -2.060000000894069
 NonOverlap_Object_0and_Obstacle_1B: y_0 + 0.5 w_0 + 2.260000000894069 C186
   <= 2.760000000894069
 NonOverlap_Object_0and_Obstacle_1F: y_0 - 0.5 w_0 - 2.260000000894069 C187
   >= -1.56000000089407
 NonOverlap_Object_0and_Obstacle_1D: z_0 + 0.5 h_0 + 2.260000000894069 C188
   <= 2.310000000894069
 NonOverlap_Object_0and_Obstacle_1U: z_0 - 0.5 h_0 - 2.260000000894069 C189
   >= -2.010000000894069
 NonOverlap_Object_0and_Obstacle_1: C184 + C185 + C186 + C187 + C188 + C189
   >= 1
 NonOverlap_Object_0and_Obstacle_2L: x_0 + 0.5 l_0 + 2.260000000894069 C190
   <= 2.940000001117587
 NonOverlap_Object_0and_Obstacle_2R: x_0 - 0.5 l_0 - 2.260000000894069 C191
   >= -1.260000001117587
 NonOverlap_Object_0and_Obstacle_2B: y_0 + 0.5 w_0 + 2.260000000894069 C192
   <= 2.260000001117587
 NonOverlap_Object_0and_Obstacle_2F: y_0 - 0.5 w_0 - 2.260000000894069 C193
   >= -1.940000001117587
 NonOverlap_Object_0and_Obstacle_2D: z_0 + 0.5 h_0 + 2.260000000894069 C194
   <= 2.260000000894069
 NonOverlap_Object_0and_Obstacle_2U: z_0 - 0.5 h_0 - 2.260000000894069 C195
   >= -1.960000000894069
 NonOverlap_Object_0and_Obstacle_2: C190 + C191 + C192 + C193 + C194 + C195
   >= 1
 NonOverlap_Object_1and_Obstacle_0L: x_1 + 0.5 l_1 + 2.260000000894069 C196
   <= 2.620000000894069
 NonOverlap_Object_1and_Obstacle_0R: x_1 - 0.5 l_1 - 2.260000000894069 C197
   >= -1.82000000089407
 NonOverlap_Object_1and_Obstacle_0B: y_1 + 0.5 w_1 + 2.260000000894069 C198
   <= 2.260000000894069
 NonOverlap_Object_1and_Obstacle_0F: y_1 - 0.5 w_1 - 2.260000000894069 C199
   >= -2.180000000894069
 NonOverlap_Object_1and_Obstacle_0D: z_1 + 0.5 h_1 + 2.260000000894069 C200
   <= 2.260000000894069
 NonOverlap_Object_1and_Obstacle_0U: z_1 - 0.5 h_1 - 2.260000000894069 C201
   >= -2.020000000894069
 NonOverlap_Object_1and_Obstacle_0: C196 + C197 + C198 + C199 + C200 + C201
   >= 1
 NonOverlap_Object_1and_Obstacle_1L: x_1 + 0.5 l_1 + 2.260000000894069 C202
   <= 2.260000000894069
 NonOverlap_Object_1and_Obstacle_1R: x_1 - 0.5 l_1 - 2.260000000894069 C203
   >= -2.060000000894069
 NonOverlap_Object_1and_Obstacle_1B: y_1 + 0.5 w_1 + 2.260000000894069 C204
   <= 2.760000000894069
 NonOverlap_Object_1and_Obstacle_1F: y_1 - 0.5 w_1 - 2.260000000894069 C205
   >= -1.56000000089407
 NonOverlap_Object_1and_Obstacle_1D: z_1 + 0.5 h_1 + 2.260000000894069 C206
   <= 2.310000000894069
 NonOverlap_Object_1and_Obstacle_1U: z_1 - 0.5 h_1 - 2.260000000894069 C207
   >= -2.010000000894069
 NonOverlap_Object_1and_Obstacle_1: C202 + C203 + C204 + C205 + C206 + C207
   >= 1
 NonOverlap_Object_1and_Obstacle_2L: x_1 + 0.5 l_1 + 2.260000000894069 C208
   <= 2.940000001117587
 NonOverlap_Object_1and_Obstacle_2R: x_1 - 0.5 l_1 - 2.260000000894069 C209
   >= -1.260000001117587
 NonOverlap_Object_1and_Obstacle_2B: y_1 + 0.5 w_1 + 2.260000000894069 C210
   <= 2.260000001117587
 NonOverlap_Object_1and_Obstacle_2F: y_1 - 0.5 w_1 - 2.260000000894069 C211
   >= -1.940000001117587
 NonOverlap_Object_1and_Obstacle_2D: z_1 + 0.5 h_1 + 2.260000000894069 C212
   <= 2.260000000894069
 NonOverlap_Object_1and_Obstacle_2U: z_1 - 0.5 h_1 - 2.260000000894069 C213
   >= -1.960000000894069
 NonOverlap_Object_1and_Obstacle_2: C208 + C209 + C210 + C211 + C212 + C213
   >= 1
 NonOverlap_Object_2and_Obstacle_0L: x_2 + 0.5 l_2 + 2.260000000894069 C214
   <= 2.620000000894069
 NonOverlap_Object_2and_Obstacle_0R: x_2 - 0.5 l_2 - 2.260000000894069 C215
   >= -1.82000000089407
 NonOverlap_Object_2and_Obstacle_0B: y_2 + 0.5 w_2 + 2.260000000894069 C216
   <= 2.260000000894069
 NonOverlap_Object_2and_Obstacle_0F: y_2 - 0.5 w_2 - 2.260000000894069 C217
   >= -2.180000000894069
 NonOverlap_Object_2and_Obstacle_0D: z_2 + 0.5 h_2 + 2.260000000894069 C218
   <= 2.260000000894069
 NonOverlap_Object_2and_Obstacle_0U: z_2 - 0.5 h_2 - 2.260000000894069 C219
   >= -2.020000000894069
 NonOverlap_Object_2and_Obstacle_0: C214 + C215 + C216 + C217 + C218 + C219
   >= 1
 NonOverlap_Object_2and_Obstacle_1L: x_2 + 0.5 l_2 + 2.260000000894069 C220
   <= 2.260000000894069
 NonOverlap_Object_2and_Obstacle_1R: x_2 - 0.5 l_2 - 2.260000000894069 C221
   >= -2.060000000894069
 NonOverlap_Object_2and_Obstacle_1B: y_2 + 0.5 w_2 + 2.260000000894069 C222
   <= 2.760000000894069
 NonOverlap_Object_2and_Obstacle_1F: y_2 - 0.5 w_2 - 2.260000000894069 C223
   >= -1.56000000089407
 NonOverlap_Object_2and_Obstacle_1D: z_2 + 0.5 h_2 + 2.260000000894069 C224
   <= 2.310000000894069
 NonOverlap_Object_2and_Obstacle_1U: z_2 - 0.5 h_2 - 2.260000000894069 C225
   >= -2.010000000894069
 NonOverlap_Object_2and_Obstacle_1: C220 + C221 + C222 + C223 + C224 + C225
   >= 1
 NonOverlap_Object_2and_Obstacle_2L: x_2 + 0.5 l_2 + 2.260000000894069 C226
   <= 2.940000001117587
 NonOverlap_Object_2and_Obstacle_2R: x_2 - 0.5 l_2 - 2.260000000894069 C227
   >= -1.260000001117587
 NonOverlap_Object_2and_Obstacle_2B: y_2 + 0.5 w_2 + 2.260000000894069 C228
   <= 2.260000001117587
 NonOverlap_Object_2and_Obstacle_2F: y_2 - 0.5 w_2 - 2.260000000894069 C229
   >= -1.940000001117587
 NonOverlap_Object_2and_Obstacle_2D: z_2 + 0.5 h_2 + 2.260000000894069 C230
   <= 2.260000000894069
 NonOverlap_Object_2and_Obstacle_2U: z_2 - 0.5 h_2 - 2.260000000894069 C231
   >= -1.960000000894069
 NonOverlap_Object_2and_Obstacle_2: C226 + C227 + C228 + C229 + C230 + C231
   >= 1
 NonOverlap_Object_3and_Obstacle_0L: x_3 + 0.5 l_3 + 2.260000000894069 C232
   <= 2.620000000894069
 NonOverlap_Object_3and_Obstacle_0R: x_3 - 0.5 l_3 - 2.260000000894069 C233
   >= -1.82000000089407
 NonOverlap_Object_3and_Obstacle_0B: y_3 + 0.5 w_3 + 2.260000000894069 C234
   <= 2.260000000894069
 NonOverlap_Object_3and_Obstacle_0F: y_3 - 0.5 w_3 - 2.260000000894069 C235
   >= -2.180000000894069
 NonOverlap_Object_3and_Obstacle_0D: z_3 + 0.5 h_3 + 2.260000000894069 C236
   <= 2.260000000894069
 NonOverlap_Object_3and_Obstacle_0U: z_3 - 0.5 h_3 - 2.260000000894069 C237
   >= -2.020000000894069
 NonOverlap_Object_3and_Obstacle_0: C232 + C233 + C234 + C235 + C236 + C237
   >= 1
 NonOverlap_Object_3and_Obstacle_1L: x_3 + 0.5 l_3 + 2.260000000894069 C238
   <= 2.260000000894069
 NonOverlap_Object_3and_Obstacle_1R: x_3 - 0.5 l_3 - 2.260000000894069 C239
   >= -2.060000000894069
 NonOverlap_Object_3and_Obstacle_1B: y_3 + 0.5 w_3 + 2.260000000894069 C240
   <= 2.760000000894069
 NonOverlap_Object_3and_Obstacle_1F: y_3 - 0.5 w_3 - 2.260000000894069 C241
   >= -1.56000000089407
 NonOverlap_Object_3and_Obstacle_1D: z_3 + 0.5 h_3 + 2.260000000894069 C242
   <= 2.310000000894069
 NonOverlap_Object_3and_Obstacle_1U: z_3 - 0.5 h_3 - 2.260000000894069 C243
   >= -2.010000000894069
 NonOverlap_Object_3and_Obstacle_1: C238 + C239 + C240 + C241 + C242 + C243
   >= 1
 NonOverlap_Object_3and_Obstacle_2L: x_3 + 0.5 l_3 + 2.260000000894069 C244
   <= 2.940000001117587
 NonOverlap_Object_3and_Obstacle_2R: x_3 - 0.5 l_3 - 2.260000000894069 C245
   >= -1.260000001117587
 NonOverlap_Object_3and_Obstacle_2B: y_3 + 0.5 w_3 + 2.260000000894069 C246
   <= 2.260000001117587
 NonOverlap_Object_3and_Obstacle_2F: y_3 - 0.5 w_3 - 2.260000000894069 C247
   >= -1.940000001117587
 NonOverlap_Object_3and_Obstacle_2D: z_3 + 0.5 h_3 + 2.260000000894069 C248
   <= 2.260000000894069
 NonOverlap_Object_3and_Obstacle_2U: z_3 - 0.5 h_3 - 2.260000000894069 C249
   >= -1.960000000894069
 NonOverlap_Object_3and_Obstacle_2: C244 + C245 + C246 + C247 + C248 + C249
   >= 1
 NonOverlap_Object_4and_Obstacle_0L: x_4 + 0.5 l_4 + 2.260000000894069 C250
   <= 2.620000000894069
 NonOverlap_Object_4and_Obstacle_0R: x_4 - 0.5 l_4 - 2.260000000894069 C251
   >= -1.82000000089407
 NonOverlap_Object_4and_Obstacle_0B: y_4 + 0.5 w_4 + 2.260000000894069 C252
   <= 2.260000000894069
 NonOverlap_Object_4and_Obstacle_0F: y_4 - 0.5 w_4 - 2.260000000894069 C253
   >= -2.180000000894069
 NonOverlap_Object_4and_Obstacle_0D: z_4 + 0.5 h_4 + 2.260000000894069 C254
   <= 2.260000000894069
 NonOverlap_Object_4and_Obstacle_0U: z_4 - 0.5 h_4 - 2.260000000894069 C255
   >= -2.020000000894069
 NonOverlap_Object_4and_Obstacle_0: C250 + C251 + C252 + C253 + C254 + C255
   >= 1
 NonOverlap_Object_4and_Obstacle_1L: x_4 + 0.5 l_4 + 2.260000000894069 C256
   <= 2.260000000894069
 NonOverlap_Object_4and_Obstacle_1R: x_4 - 0.5 l_4 - 2.260000000894069 C257
   >= -2.060000000894069
 NonOverlap_Object_4and_Obstacle_1B: y_4 + 0.5 w_4 + 2.260000000894069 C258
   <= 2.760000000894069
 NonOverlap_Object_4and_Obstacle_1F: y_4 - 0.5 w_4 - 2.260000000894069 C259
   >= -1.56000000089407
 NonOverlap_Object_4and_Obstacle_1D: z_4 + 0.5 h_4 + 2.260000000894069 C260
   <= 2.310000000894069
 NonOverlap_Object_4and_Obstacle_1U: z_4 - 0.5 h_4 - 2.260000000894069 C261
   >= -2.010000000894069
 NonOverlap_Object_4and_Obstacle_1: C256 + C257 + C258 + C259 + C260 + C261
   >= 1
 NonOverlap_Object_4and_Obstacle_2L: x_4 + 0.5 l_4 + 2.260000000894069 C262
   <= 2.940000001117587
 NonOverlap_Object_4and_Obstacle_2R: x_4 - 0.5 l_4 - 2.260000000894069 C263
   >= -1.260000001117587
 NonOverlap_Object_4and_Obstacle_2B: y_4 + 0.5 w_4 + 2.260000000894069 C264
   <= 2.260000001117587
 NonOverlap_Object_4and_Obstacle_2F: y_4 - 0.5 w_4 - 2.260000000894069 C265
   >= -1.940000001117587
 NonOverlap_Object_4and_Obstacle_2D: z_4 + 0.5 h_4 + 2.260000000894069 C266
   <= 2.260000000894069
 NonOverlap_Object_4and_Obstacle_2U: z_4 - 0.5 h_4 - 2.260000000894069 C267
   >= -1.960000000894069
 NonOverlap_Object_4and_Obstacle_2: C262 + C263 + C264 + C265 + C266 + C267
   >= 1
 NonOverlap_Object_5and_Obstacle_0L: x_5 + 0.5 l_5 + 2.260000000894069 C268
   <= 2.620000000894069
 NonOverlap_Object_5and_Obstacle_0R: x_5 - 0.5 l_5 - 2.260000000894069 C269
   >= -1.82000000089407
 NonOverlap_Object_5and_Obstacle_0B: y_5 + 0.5 w_5 + 2.260000000894069 C270
   <= 2.260000000894069
 NonOverlap_Object_5and_Obstacle_0F: y_5 - 0.5 w_5 - 2.260000000894069 C271
   >= -2.180000000894069
 NonOverlap_Object_5and_Obstacle_0D: z_5 + 0.5 h_5 + 2.260000000894069 C272
   <= 2.260000000894069
 NonOverlap_Object_5and_Obstacle_0U: z_5 - 0.5 h_5 - 2.260000000894069 C273
   >= -2.020000000894069
 NonOverlap_Object_5and_Obstacle_0: C268 + C269 + C270 + C271 + C272 + C273
   >= 1
 NonOverlap_Object_5and_Obstacle_1L: x_5 + 0.5 l_5 + 2.260000000894069 C274
   <= 2.260000000894069
 NonOverlap_Object_5and_Obstacle_1R: x_5 - 0.5 l_5 - 2.260000000894069 C275
   >= -2.060000000894069
 NonOverlap_Object_5and_Obstacle_1B: y_5 + 0.5 w_5 + 2.260000000894069 C276
   <= 2.760000000894069
 NonOverlap_Object_5and_Obstacle_1F: y_5 - 0.5 w_5 - 2.260000000894069 C277
   >= -1.56000000089407
 NonOverlap_Object_5and_Obstacle_1D: z_5 + 0.5 h_5 + 2.260000000894069 C278
   <= 2.310000000894069
 NonOverlap_Object_5and_Obstacle_1U: z_5 - 0.5 h_5 - 2.260000000894069 C279
   >= -2.010000000894069
 NonOverlap_Object_5and_Obstacle_1: C274 + C275 + C276 + C277 + C278 + C279
   >= 1
 NonOverlap_Object_5and_Obstacle_2L: x_5 + 0.5 l_5 + 2.260000000894069 C280
   <= 2.940000001117587
 NonOverlap_Object_5and_Obstacle_2R: x_5 - 0.5 l_5 - 2.260000000894069 C281
   >= -1.260000001117587
 NonOverlap_Object_5and_Obstacle_2B: y_5 + 0.5 w_5 + 2.260000000894069 C282
   <= 2.260000001117587
 NonOverlap_Object_5and_Obstacle_2F: y_5 - 0.5 w_5 - 2.260000000894069 C283
   >= -1.940000001117587
 NonOverlap_Object_5and_Obstacle_2D: z_5 + 0.5 h_5 + 2.260000000894069 C284
   <= 2.260000000894069
 NonOverlap_Object_5and_Obstacle_2U: z_5 - 0.5 h_5 - 2.260000000894069 C285
   >= -1.960000000894069
 NonOverlap_Object_5and_Obstacle_2: C280 + C281 + C282 + C283 + C284 + C285
   >= 1
 NonOverlap_Object_6and_Obstacle_0L: x_6 + 0.5 l_6 + 2.260000000894069 C286
   <= 2.620000000894069
 NonOverlap_Object_6and_Obstacle_0R: x_6 - 0.5 l_6 - 2.260000000894069 C287
   >= -1.82000000089407
 NonOverlap_Object_6and_Obstacle_0B: y_6 + 0.5 w_6 + 2.260000000894069 C288
   <= 2.260000000894069
 NonOverlap_Object_6and_Obstacle_0F: y_6 - 0.5 w_6 - 2.260000000894069 C289
   >= -2.180000000894069
 NonOverlap_Object_6and_Obstacle_0D: z_6 + 0.5 h_6 + 2.260000000894069 C290
   <= 2.260000000894069
 NonOverlap_Object_6and_Obstacle_0U: z_6 - 0.5 h_6 - 2.260000000894069 C291
   >= -2.020000000894069
 NonOverlap_Object_6and_Obstacle_0: C286 + C287 + C288 + C289 + C290 + C291
   >= 1
 NonOverlap_Object_6and_Obstacle_1L: x_6 + 0.5 l_6 + 2.260000000894069 C292
   <= 2.260000000894069
 NonOverlap_Object_6and_Obstacle_1R: x_6 - 0.5 l_6 - 2.260000000894069 C293
   >= -2.060000000894069
 NonOverlap_Object_6and_Obstacle_1B: y_6 + 0.5 w_6 + 2.260000000894069 C294
   <= 2.760000000894069
 NonOverlap_Object_6and_Obstacle_1F: y_6 - 0.5 w_6 - 2.260000000894069 C295
   >= -1.56000000089407
 NonOverlap_Object_6and_Obstacle_1D: z_6 + 0.5 h_6 + 2.260000000894069 C296
   <= 2.310000000894069
 NonOverlap_Object_6and_Obstacle_1U: z_6 - 0.5 h_6 - 2.260000000894069 C297
   >= -2.010000000894069
 NonOverlap_Object_6and_Obstacle_1: C292 + C293 + C294 + C295 + C296 + C297
   >= 1
 NonOverlap_Object_6and_Obstacle_2L: x_6 + 0.5 l_6 + 2.260000000894069 C298
   <= 2.940000001117587
 NonOverlap_Object_6and_Obstacle_2R: x_6 - 0.5 l_6 - 2.260000000894069 C299
   >= -1.260000001117587
 NonOverlap_Object_6and_Obstacle_2B: y_6 + 0.5 w_6 + 2.260000000894069 C300
   <= 2.260000001117587
 NonOverlap_Object_6and_Obstacle_2F: y_6 - 0.5 w_6 - 2.260000000894069 C301
   >= -1.940000001117587
 NonOverlap_Object_6and_Obstacle_2D: z_6 + 0.5 h_6 + 2.260000000894069 C302
   <= 2.260000000894069
 NonOverlap_Object_6and_Obstacle_2U: z_6 - 0.5 h_6 - 2.260000000894069 C303
   >= -1.960000000894069
 NonOverlap_Object_6and_Obstacle_2: C298 + C299 + C300 + C301 + C302 + C303
   >= 1
 NonOverlap_Object_7and_Obstacle_0L: x_7 + 0.5 l_7 + 2.260000000894069 C304
   <= 2.620000000894069
 NonOverlap_Object_7and_Obstacle_0R: x_7 - 0.5 l_7 - 2.260000000894069 C305
   >= -1.82000000089407
 NonOverlap_Object_7and_Obstacle_0B: y_7 + 0.5 w_7 + 2.260000000894069 C306
   <= 2.260000000894069
 NonOverlap_Object_7and_Obstacle_0F: y_7 - 0.5 w_7 - 2.260000000894069 C307
   >= -2.180000000894069
 NonOverlap_Object_7and_Obstacle_0D: z_7 + 0.5 h_7 + 2.260000000894069 C308
   <= 2.260000000894069
 NonOverlap_Object_7and_Obstacle_0U: z_7 - 0.5 h_7 - 2.260000000894069 C309
   >= -2.020000000894069
 NonOverlap_Object_7and_Obstacle_0: C304 + C305 + C306 + C307 + C308 + C309
   >= 1
 NonOverlap_Object_7and_Obstacle_1L: x_7 + 0.5 l_7 + 2.260000000894069 C310
   <= 2.260000000894069
 NonOverlap_Object_7and_Obstacle_1R: x_7 - 0.5 l_7 - 2.260000000894069 C311
   >= -2.060000000894069
 NonOverlap_Object_7and_Obstacle_1B: y_7 + 0.5 w_7 + 2.260000000894069 C312
   <= 2.760000000894069
 NonOverlap_Object_7and_Obstacle_1F: y_7 - 0.5 w_7 - 2.260000000894069 C313
   >= -1.56000000089407
 NonOverlap_Object_7and_Obstacle_1D: z_7 + 0.5 h_7 + 2.260000000894069 C314
   <= 2.310000000894069
 NonOverlap_Object_7and_Obstacle_1U: z_7 - 0.5 h_7 - 2.260000000894069 C315
   >= -2.010000000894069
 NonOverlap_Object_7and_Obstacle_1: C310 + C311 + C312 + C313 + C314 + C315
   >= 1
 NonOverlap_Object_7and_Obstacle_2L: x_7 + 0.5 l_7 + 2.260000000894069 C316
   <= 2.940000001117587
 NonOverlap_Object_7and_Obstacle_2R: x_7 - 0.5 l_7 - 2.260000000894069 C317
   >= -1.260000001117587
 NonOverlap_Object_7and_Obstacle_2B: y_7 + 0.5 w_7 + 2.260000000894069 C318
   <= 2.260000001117587
 NonOverlap_Object_7and_Obstacle_2F: y_7 - 0.5 w_7 - 2.260000000894069 C319
   >= -1.940000001117587
 NonOverlap_Object_7and_Obstacle_2D: z_7 + 0.5 h_7 + 2.260000000894069 C320
   <= 2.260000000894069
 NonOverlap_Object_7and_Obstacle_2U: z_7 - 0.5 h_7 - 2.260000000894069 C321
   >= -1.960000000894069
 NonOverlap_Object_7and_Obstacle_2: C316 + C317 + C318 + C319 + C320 + C321
   >= 1
 Boundary_Object_0_Right_eq: x_0 + 0.5 l_0 = 0.99
 Boundary_Object_0_Right_ieq: y_0 >= 0.31
 Boundary_Object_0_Right_ieqq: y_0 <= 0.99
 Boundary_Object_4_Left_eq: x_4 - 0.5 l_4 = 0.01
 Boundary_Object_4_Left_ieq: y_4 >= 0.01
 Boundary_Object_4_Left_ieqq: y_4 <= 0.99
 TopLeft_Corner_of_Object_3eqa: C322 = 1
 TopLeft_Corner_of_Object_3eqb: x_3 - 0.5 l_3 - 0.01 C322 = 0
 TopLeft_Corner_of_Object_3eqc: y_3 + 0.5 w_3 - 0.99 C322 = 0
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
 C207 C208 C209 C210 C211 C212 C213 C214 C215 C216 C217 C218 C219 C220 C221
 C222 C223 C224 C225 C226 C227 C228 C229 C230 C231 C232 C233 C234 C235 C236
 C237 C238 C239 C240 C241 C242 C243 C244 C245 C246 C247 C248 C249 C250 C251
 C252 C253 C254 C255 C256 C257 C258 C259 C260 C261 C262 C263 C264 C265 C266
 C267 C268 C269 C270 C271 C272 C273 C274 C275 C276 C277 C278 C279 C280 C281
 C282 C283 C284 C285 C286 C287 C288 C289 C290 C291 C292 C293 C294 C295 C296
 C297 C298 C299 C300 C301 C302 C303 C304 C305 C306 C307 C308 C309 C310 C311
 C312 C313 C314 C315 C316 C317 C318 C319 C320 C321 C322
End
