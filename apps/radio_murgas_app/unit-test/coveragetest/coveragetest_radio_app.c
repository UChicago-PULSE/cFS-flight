/************************************************************************
 * NASA Docket No. GSC-18,719-1, and identified as "core Flight System: Bootes"
 *
 * Copyright (c) 2020 United States Government as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License. You may obtain
 * a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ************************************************************************/

/*
** File: coveragetest_radio_app.c
**
** Purpose:
** Coverage Unit Test cases for the RADIO Application
**
** Notes:
** This implements various test cases to exercise code
** paths through key functions defined in the RADIO application.
*/

/*
 * Includes
 */

#include <string.h>
#include "radio_app_coveragetest_common.h"
#include "ut_radio_app.h"
#include "bus_comms_msg.h"
#include "bus_comms_msgids.h"

/*
 * Unit test check event hook information
 */
typedef struct
{
    uint16      ExpectedEvent;
    uint32      MatchCount;
    const char *ExpectedFormat;
} UT_CheckEvent_t;

/*
 * An example hook function to check for a specific event.
 */
static int32 UT_CheckEvent_Hook(void *UserObj, int32 StubRetcode, uint32 CallCount, const UT_StubContext_t *Context,
                                va_list va)
{
    UT_CheckEvent_t *State = UserObj;
    uint16           EventId;
    const char *     Spec;

    if (Context->ArgCount > 0)
    {
        EventId = UT_Hook_GetArgValueByName(Context, "EventID", uint16);
        if (EventId == State->ExpectedEvent)
        {
            if (State->ExpectedFormat != NULL)
            {
                Spec = UT_Hook_GetArgValueByName(Context, "Spec", const char *);
                if (Spec != NULL)
                {
                    if (strcmp(Spec, State->ExpectedFormat) == 0)
                    {
                        ++State->MatchCount;
                    }
                }
            }
            else
            {
                ++State->MatchCount;
            }
        }
    }

    return 0;
}

/* Macro to get expected event name */
#define UT_CHECKEVENT_SETUP(Evt, ExpectedEvent, ExpectedFormat) \
    UT_CheckEvent_Setup_Impl(Evt, ExpectedEvent, #ExpectedEvent, ExpectedFormat)

/*
 * Helper function to set up for event checking
 * This attaches the hook function to CFE_EVS_SendEvent
 */
static void UT_CheckEvent_Setup_Impl(UT_CheckEvent_t *Evt, uint16 ExpectedEvent, const char *EventName,
                                     const char *ExpectedFormat)
{
    if (ExpectedFormat == NULL)
    {
        UtPrintf("CheckEvent will match: %s(%u)", EventName, ExpectedEvent);
    }
    else
    {
        UtPrintf("CheckEvent will match: %s(%u), \"%s\"", EventName, ExpectedEvent, ExpectedFormat);
    }
    memset(Evt, 0, sizeof(*Evt));
    Evt->ExpectedEvent  = ExpectedEvent;
    Evt->ExpectedFormat  = ExpectedFormat;
    UT_SetVaHookFunction(UT_KEY(CFE_EVS_SendEvent), UT_CheckEvent_Hook, Evt);
}

/*
**********************************************************************************
**          TEST CASE FUNCTIONS
**********************************************************************************
*/

void Test_RADIO_APP_Init(void)
{
    /*
     * Test Case For:
     * int32 RADIO_APP_Init(void)
     */

    int32 status;

    /*
     * Nominal case - all dependent calls succeed
     */
    status = RADIO_APP_Init();
    UtAssert_INT32_EQ(status, CFE_SUCCESS);

    /*
     * Verify that CFE_EVS_Register was called
     */
    UtAssert_STUB_COUNT(CFE_EVS_Register, 1);

    /*
     * Verify that CFE_SB_CreatePipe was called
     */
    UtAssert_STUB_COUNT(CFE_SB_CreatePipe, 1);

    /*
     * Verify that CFE_SB_Subscribe was called (for CMD_MID and SEND_HK_MID)
     */
    UtAssert_STUB_COUNT(CFE_SB_Subscribe, 2);

    /*
     * Error case - CFE_EVS_Register fails
     */
    UT_SetDeferredRetcode(UT_KEY(CFE_EVS_Register), 1, CFE_EVS_INVALID_PARAMETER);
    status = RADIO_APP_Init();
    UtAssert_INT32_EQ(status, CFE_EVS_INVALID_PARAMETER);
}

void Test_RADIO_APP_Noop(void)
{
    /*
     * Test Case For:
     * int32 RADIO_APP_Noop(const RADIO_APP_NoopCmd_t *Msg)
     */

    RADIO_APP_NoopCmd_t NoopCmd;
    int32               status;
    UT_CheckEvent_t     EventTest;

    memset(&NoopCmd, 0, sizeof(NoopCmd));

    /*
     * Set up event checking
     */
    UT_CHECKEVENT_SETUP(&EventTest, RADIO_APP_COMMANDNOOP_INF_EID, NULL);

    /*
     * Nominal case
     */
    status = RADIO_APP_Noop((const RADIO_APP_NoopCmd_t *)&NoopCmd);
    UtAssert_INT32_EQ(status, CFE_SUCCESS);

    /*
     * Verify command counter was incremented
     */
    UtAssert_UINT8_EQ(RADIO_APP_Data.CmdCounter, 1);

    /*
     * Verify event was sent
     */
    UtAssert_UINT32_EQ(EventTest.MatchCount, 1);
}

void Test_RADIO_APP_ResetCounters(void)
{
    /*
     * Test Case For:
     * int32 RADIO_APP_ResetCounters(const RADIO_APP_ResetCountersCmd_t *Msg)
     */

    RADIO_APP_ResetCountersCmd_t ResetCmd;
    int32                        status;
    UT_CheckEvent_t              EventTest;

    memset(&ResetCmd, 0, sizeof(ResetCmd));

    /*
     * Set initial counter values
     */
    RADIO_APP_Data.CmdCounter = 5;
    RADIO_APP_Data.ErrCounter = 3;

    /*
     * Set up event checking
     */
    UT_CHECKEVENT_SETUP(&EventTest, RADIO_APP_COMMANDRESET_INF_EID, NULL);

    /*
     * Nominal case
     */
    status = RADIO_APP_ResetCounters((const RADIO_APP_ResetCountersCmd_t *)&ResetCmd);
    UtAssert_INT32_EQ(status, CFE_SUCCESS);

    /*
     * Verify counters were reset
     */
    UtAssert_UINT8_EQ(RADIO_APP_Data.CmdCounter, 0);
    UtAssert_UINT8_EQ(RADIO_APP_Data.ErrCounter, 0);

    /*
     * Verify event was sent
     */
    UtAssert_UINT32_EQ(EventTest.MatchCount, 1);
}

void Test_RADIO_APP_VerifyCmdLength(void)
{
    /*
     * Test Case For:
     * bool RADIO_APP_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
     */

    CFE_MSG_Message_t Msg;
    bool              result;

    /*
     * Nominal case - correct length
     */
    CFE_MSG_Init(&Msg, CFE_SB_ValueToMsgId(RADIO_APP_CMD_MID), sizeof(RADIO_APP_NoopCmd_t));
    result = RADIO_APP_VerifyCmdLength(&Msg, sizeof(RADIO_APP_NoopCmd_t));
    UtAssert_BOOL_TRUE(result);

    /*
     * Error case - incorrect length
     */
    CFE_MSG_Init(&Msg, CFE_SB_ValueToMsgId(RADIO_APP_CMD_MID), sizeof(RADIO_APP_NoopCmd_t) - 1);
    result = RADIO_APP_VerifyCmdLength(&Msg, sizeof(RADIO_APP_NoopCmd_t));
    UtAssert_BOOL_FALSE(result);

    /*
     * Verify error counter was incremented
     */
    UtAssert_UINT8_EQ(RADIO_APP_Data.ErrCounter, 1);
}

void Test_RADIO_APP_ConfigureRadio(void)
{
    /*
     * Test Case For:
     * int32 RADIO_APP_ConfigureRadio(const RADIO_APP_ConfigureCmd_t *Msg)
     */

    RADIO_APP_ConfigureCmd_t ConfigCmd;
    int32                     status;
    UT_CheckEvent_t           EventTest;
    const char *              test_command = "set/configure freq=437000000";

    memset(&ConfigCmd, 0, sizeof(ConfigCmd));
    ConfigCmd.CommandLength = strlen(test_command);
    strncpy(ConfigCmd.Command, test_command, sizeof(ConfigCmd.Command) - 1);
    ConfigCmd.Command[sizeof(ConfigCmd.Command) - 1] = 0;

    /*
     * Set up event checking
     */
    UT_CHECKEVENT_SETUP(&EventTest, RADIO_APP_CONFIGURE_INF_EID, NULL);

    /*
     * Nominal case - command forwarded successfully
     */
    UT_SetDefaultReturnValue(UT_KEY(CFE_SB_TransmitMsg), CFE_SUCCESS);
    status = RADIO_APP_ConfigureRadio((const RADIO_APP_ConfigureCmd_t *)&ConfigCmd);
    UtAssert_INT32_EQ(status, CFE_SUCCESS);

    /*
     * Verify command counter was incremented
     */
    UtAssert_UINT8_EQ(RADIO_APP_Data.CmdCounter, 1);

    /*
     * Verify event was sent
     */
    UtAssert_UINT32_EQ(EventTest.MatchCount, 1);

    /*
     * Verify CFE_SB_TransmitMsg was called
     */
    UtAssert_STUB_COUNT(CFE_SB_TransmitMsg, 1);

    /*
     * Error case - command length exceeds maximum
     */
    ConfigCmd.CommandLength = 257;
    status                  = RADIO_APP_ConfigureRadio((const RADIO_APP_ConfigureCmd_t *)&ConfigCmd);
    UtAssert_INT32_EQ(status, CFE_SB_BAD_ARGUMENT);
    UtAssert_UINT8_EQ(RADIO_APP_Data.ErrCounter, 1);
}

/*
 * Setup function prior to every test
 */
void Radio_UT_Setup(void)
{
    UT_ResetState(0);
}

/*
 * Teardown function after every test
 */
void Radio_UT_TearDown(void) {}

/*
 * Register the test cases to execute with the unit test tool
 */
void UtTest_Setup(void)
{
    ADD_TEST(RADIO_APP_Init);
    ADD_TEST(RADIO_APP_Noop);
    ADD_TEST(RADIO_APP_ResetCounters);
    ADD_TEST(RADIO_APP_VerifyCmdLength);
    ADD_TEST(RADIO_APP_ConfigureRadio);
}
