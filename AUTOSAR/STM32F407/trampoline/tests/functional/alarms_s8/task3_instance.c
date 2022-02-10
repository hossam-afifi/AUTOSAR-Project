/**
 * @file alarms_s8/task3_instance.c
 *
 * @section desc File description
 *
 * @section copyright Copyright
 *
 * Trampoline Test Suite
 *
 * Trampoline Test Suite is copyright (c) IRCCyN 2005-2007
 * Trampoline Test Suite is protected by the French intellectual property law.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * @section infos File informations
 *
 * $Date$
 * $Rev$
 * $Author$
 * $URL$
 */

/*Instance of task t3*/

#include "tpl_os.h"

DeclareAlarm(Alarm1);
DeclareTask(t2);

void WaitActivationOneShotAlarm(AlarmType Alarm);

/*test case:test the reaction of the system called with 
an activation of a task*/
static void test_t3_instance(void)
{
	
	StatusType result_inst_1, result_inst_3, result_inst_4;
	TaskStateType result_inst_2;
	
	SCHEDULING_CHECK_INIT(3);
	result_inst_1 = SetRelAlarm(Alarm1, 2, 0);
	SCHEDULING_CHECK_AND_EQUAL_INT(3,E_OK, result_inst_1);
	
	WaitActivationOneShotAlarm(Alarm1);
	
	SCHEDULING_CHECK_INIT(4);
	result_inst_3 = GetTaskState(t2, &result_inst_2);
	SCHEDULING_CHECK_AND_EQUAL_INT_FIRST(4,READY, result_inst_2);
	SCHEDULING_CHECK_AND_EQUAL_INT(4,E_OK, result_inst_3);
	
	SCHEDULING_CHECK_INIT(5);
	result_inst_4 = TerminateTask();
	SCHEDULING_CHECK_AND_EQUAL_INT(5,E_OK, result_inst_4);
	
}

/*create the test suite with all the test cases*/
TestRef AlarmsTest_seq8_t3_instance(void)
{
	EMB_UNIT_TESTFIXTURES(fixtures) {
		new_TestFixture("test_t3_instance",test_t3_instance)
	};
	EMB_UNIT_TESTCALLER(AlarmsTest,"AlarmsTest_sequence8",NULL,NULL,fixtures);

	return (TestRef)&AlarmsTest;
}

/* End of file alarms_s8/task3_instance.c */
