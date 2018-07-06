//=============================================================================
// Copyright � 2017 FLIR Integrated Imaging Solutions, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

#ifndef SPINNAKER_GENAPI_ENUMENTRYNODE_H
#define SPINNAKER_GENAPI_ENUMENTRYNODE_H

#include "SpinnakerPlatform.h"
#include "Types.h"
#include "Base.h"
#include "GCString.h"
#include "ISelector.h"
#include "INode.h"
#include "ValueNode.h"
#include "IEnumEntry.h"

#pragma warning ( push )
#pragma warning( disable : 4250 ) // C4250 - 'class1' : inherits 'class2::member' via dominance
#pragma warning ( disable : 4251 ) // XXX needs to have dll-interface to be used by clients of class YYY
#pragma warning( disable: 4275 ) // non dll-interface structXXX used as base

namespace Spinnaker
{
    namespace GenApi
    {
        /**
        *  @defgroup SpinnakerGenApiClasses Spinnaker GenApi Classes
        */
        /*@{*/
        
        /**
        *  @defgroup EnumEntryNode_h EnumEntryNode Class
        */
        /*@{*/

        /**
        * @brief Interface for string properties
        */
        class SPINNAKER_API EnumEntryNode : virtual public IEnumEntry, virtual public ValueNode
        {
        public:
            struct NodeImpl;
            EnumEntryNode();

            EnumEntryNode(std::shared_ptr<Node::NodeImpl> pEnumEntry);

            virtual ~EnumEntryNode();

            /**
            * Get numeric enum value
            */
            virtual int64_t GetValue();

            /**
            * Get symbolic enum value
            */
            virtual GenICam::gcstring GetSymbolic()const;

            /**
            * Get double number associated with the entry
            */
            virtual double GetNumericValue();

            /**
            * Indicates if the corresponding EnumEntry is self clearing
            */
            virtual bool IsSelfClearing();

            /**
            * overload SetReference for EnumEntry
            */
            virtual void SetReference(INode* pBase);


        private:

            std::shared_ptr<Node::NodeImpl> m_pEnumEntry;

        };

        typedef EnumEntryNode CEnumEntryRef;

        /*@}*/
        /*@}*/
    }
}

#endif // SPINNAKER_GENAPI_ENUMENTRYNODE_H