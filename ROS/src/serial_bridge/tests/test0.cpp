#include <gtest/gtest.h>
#include "utils.hpp"
#include "protocol.hpp"


/** ProtocolInteger:
    * @brief Wrapper class to store integers using allways big-endian.
    * @details The uint32_t containing the value is allways stored as big-endian
*/

TEST(ProtocolInteger, OverloadAssignOperator){
    /**
     * @test: When an ProtocolInteger is 
     * assigned whit an uint32_t the number 
     * is stored in memory as big-endian.
    */

    uahruart::utils::ProtocolInteger n;
    unsigned char* first_byte;
    unsigned char* second_byte;
    unsigned char* third_byte;
    unsigned char* fourth_byte;

    // Format the pointers such that
    // they point to the correponding
    // bytes of the ProtocolInteger
    first_byte  = (unsigned char *)&n;
    second_byte = first_byte;
    third_byte  = ++second_byte;
    fourth_byte = ++third_byte;
    fourth_byte++;


    n = 0x00;
    EXPECT_EQ(*first_byte,  0x00);
    EXPECT_EQ(*second_byte, 0x00);
    EXPECT_EQ(*third_byte,  0x00);
    EXPECT_EQ(*fourth_byte, 0x00);

    n = 0xEA;
    EXPECT_EQ(*first_byte,  0xEA);
    EXPECT_EQ(*second_byte, 0x00);
    EXPECT_EQ(*third_byte,  0x00);
    EXPECT_EQ(*fourth_byte, 0x00);

    n = 0x0100;
    EXPECT_EQ(*first_byte,  0x00);
    EXPECT_EQ(*second_byte, 0x01);
    EXPECT_EQ(*third_byte,  0x00);
    EXPECT_EQ(*fourth_byte, 0x00);

    n = 0x010000DD;
    EXPECT_EQ(*first_byte,  0xDD);
    EXPECT_EQ(*second_byte, 0x00);
    EXPECT_EQ(*third_byte,  0x00);
    EXPECT_EQ(*fourth_byte, 0x01);

    n = 0x01234567;
    EXPECT_EQ(*first_byte,  0x67);
    EXPECT_EQ(*second_byte, 0x45);
    EXPECT_EQ(*third_byte,  0x23);
    EXPECT_EQ(*fourth_byte, 0X01);
}

TEST(ProtocolInteger, OverloadUint32AssingOperator){
    /**
      * @test: When an ProtocolInteger is 
      * assigned whit an uint32_t the number 
      * is stored in memory as big-endian.
      * If a ProtocolInteger is assigned to
      * an uint32_t the number must be passed
      * from the Big-Endian format of the 
      * ProtocolInteger to the processors
      * Endian Format and must be interpreted
      * in the correct way.
    */

    uahruart::utils::ProtocolInteger n;
    uahruart::uint32_t assinged = 1600;
    n = 4;
    assinged = n;
    EXPECT_EQ(assinged, (uahruart::uint32_t)4);

    n = 256;
    assinged = n;
    EXPECT_EQ(assinged, (uahruart::uint32_t)256);

    n = 1340;
    assinged = n;
    EXPECT_EQ(assinged, (uahruart::uint32_t)1340);
}


void rx_send_bytes(char* msg, const unsigned int len) {
    return;
}



class PortTest : public ::testing::Test {
 protected:
  void SetUp() override {
    uahruart::Instance rx(rx_send_bytes); 
  }
  //uahruart::Instance rx();
  // void TearDown() override {}
};

//TEST_F(PortTest, IsEmptyInitially) {
//    rx.parse_byte('a')
//}




//TEST(SwitchEndianes, SwitchZero){
//    uahruart::uint32_t *n = 0;
//    uahruart::uint32_t switched_n = 1500;
//
//    switched_n = uahruart::utils::ProtocolInteger::(0);
//
//    EXPECT_EQ(switch_endianess(n), n)
//
//}


