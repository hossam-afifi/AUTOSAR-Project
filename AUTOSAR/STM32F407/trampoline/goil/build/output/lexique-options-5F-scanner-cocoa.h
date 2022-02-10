//--- START OF USER ZONE 1


//--- END OF USER ZONE 1

#import "OC_Lexique.h"

//----------------------------------------------------------------------------------------------------------------------
//
//                    E X T E R N    R O U T I N E S                                             
//
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
//
//                    E X T E R N    F U N C T I O N S                                           
//
//----------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------
//
//         T E R M I N A L    S Y M B O L    E N U M E R A T I O N                               
//
//----------------------------------------------------------------------------------------------------------------------

enum {options_scanner_1_,
  options_scanner_1_idf,
  options_scanner_1_string,
  options_scanner_1_uint_5F_number,
  options_scanner_1_float_5F_number,
  options_scanner_1__3D_,
  options_scanner_1__2C_,
  options_scanner_1__2D_,
  options_scanner_1__28_,
  options_scanner_1__29_
} ;

//----------------------------------------------------------------------------------------------------------------------
//
//        U N I C O D E    T E S T    F U N C T I O N S                                          
//
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
//
//                     S C A N N E R    C L A S S                                                
//
//----------------------------------------------------------------------------------------------------------------------

@interface OC_Lexique_options_scanner : OC_Lexique {
//--- Attributes
  @private double mLexicalAttribute_floatNumber ;
  @private UInt64 mLexicalAttribute_integerNumber ;
  @private NSMutableString * mLexicalAttribute_key ;
  @private NSMutableString * mLexicalAttribute_number ;
  @private NSMutableString * mLexicalAttribute_string ;

}

- (NSUInteger) terminalVocabularyCount ;

- (NSUInteger) styleIndexForTerminal: (NSInteger) inTerminal ;

- (void) parseLexicalTokenForLexicalColoring ;

- (NSUInteger) styleCount ;

- (NSString *) styleNameForStyleIndex: (NSInteger) inIndex ;

- (NSString *) styleIdentifierForStyleIndex: (NSInteger) inIndex ;

- (NSString *) indexingDirectory ;

- (NSArray *) indexingTitles ; // Array of NSString

- (BOOL) isTemplateLexique ;

- (BOOL) atomicSelectionForToken: (NSUInteger) inTokenIndex ;

@end

//----------------------------------------------------------------------------------------------------------------------
//--- START OF USER ZONE 2


//--- END OF USER ZONE 2

