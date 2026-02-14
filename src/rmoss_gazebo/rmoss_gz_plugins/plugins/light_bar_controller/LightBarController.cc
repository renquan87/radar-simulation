/*******************************************************************************
 *  Copyright (c) 2020 robomaster-oss, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
#include <mutex>
#include <map>
#include <ignition/common/Util.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Material.hh>

// SdfEntityCreator removed – unused and its EventManager callbacks can cause
// the renderer to double-process visuals, triggering the duplicate-name crash.
#include <sdf/Visual.hh>

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/Conversions.hh>

#include "LightBarController.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;


sdf::Material GetMaterial(int state){
    sdf::Material m;
    ignition::math::Color color;
    if(state == 0){
        color.Set(1,1,1,1);
    }else if(state == 1){
        color.Set(1,0,0,1);
    }else if(state == 2){
        color.Set(0,0,1,1);
    }else if(state == 3){
        color.Set(1,1,0,1);
    }else{
        color.Set(1,1,1,1);
    }
    m.SetAmbient(color);
    m.SetDiffuse(color);
    m.SetSpecular(color);
    if(state != 0){
        m.SetEmissive(color);
    }
    return m;
}


struct VisualEntityInfo {
    Entity entity;
    Entity parentEntity;
    sdf::Visual visualSdf;
    int state;
    VisualEntityInfo(Entity _entity,Entity parentEntity,sdf::Visual &_visualSdf,int _state)
        : entity(_entity)
        , parentEntity(parentEntity)
        , visualSdf(_visualSdf)
        , state(_state)
    {
    }
};

class ignition::gazebo::systems::LightBarControllerPrivate
{
public:
    void OnCmd(const ignition::msgs::Int32 &_msg);
    void Init(ignition::gazebo::EntityComponentManager &_ecm);
    void UpdateVisualEnitiies(ignition::gazebo::EntityComponentManager &_ecm);

public:
    transport::Node node;
    //model
    Model model{kNullEntity};
    sdf::Model modelSdf;
    std::vector<std::string> linkVisuals;
    std::vector<VisualEntityInfo> visualEntityInfos;
    bool isInit{false};
    bool isDone{true};
    int initFrameCount{0};
    // Number of PreUpdate frames to skip before allowing material changes,
    // giving the renderer time to finish creating all visuals.
    static constexpr int kInitDelayFrames = 20;
    // cmd
    // 0:no light, 1:red light, 2:blue light, 3:yellow light, 4:white light
    int targetState{0};
    bool change{false};
    std::mutex targetMutex;
};

/******************implementation for LightBarController************************/
LightBarController::LightBarController() : dataPtr(std::make_unique<LightBarControllerPrivate>())
{
}

void LightBarController::Configure(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager & _eventMgr)
{
    this->dataPtr->model = Model(_entity);
    if (!this->dataPtr->model.Valid(_ecm))
    {
        ignerr << "LightBarController plugin should be attached to a model entity. Failed to initialize." << std::endl;
        return;
    }
    // Get params from SDF
    std::string controller_name = "light_bar_controller";
    if (_sdf->HasElement("controller_name"))
    {
        controller_name = _sdf->Get<std::string>("controller_name");
    }
    
    if (_sdf->HasElement("initial_color"))
    {
        std::map<std::string,int> color_map{{"none",0},{"red",1},{"blue",2},{"yellow",3},{"white",4}};
        auto color = _sdf->Get<std::string>("initial_color");
        if(color_map.find(color)!=color_map.end()){
            this->dataPtr->targetState = color_map[color];
            // Only schedule a material change if the color is not "none" (0).
            // "none" matches the default SDF material, so no update is needed
            // and skipping it avoids a race with the renderer during startup.
            if(this->dataPtr->targetState != 0){
                this->dataPtr->change = true;
            }
        }else{
            ignwarn << "LightBarController color [" << color << "] is invalid." << std::endl;
        }
    }
    // link_visual
    auto ptr = const_cast<sdf::Element *>(_sdf.get());
    sdf::ElementPtr sdfElem = ptr->GetElement("link_visual");
    while (sdfElem)
    {
        auto path = sdfElem->Get<std::string>();
        this->dataPtr->linkVisuals.push_back(std::move(path));
        sdfElem = sdfElem->GetNextElement("link_visual");
    }
    // Model Sdf
    this->dataPtr->modelSdf = _ecm.Component<components::ModelSdf>(_entity)->Data();
    // Subscribe to commands
    std::string topic{this->dataPtr->model.Name(_ecm) +"/"+controller_name+ "/set_state"};
    this->dataPtr->node.Subscribe(topic, &LightBarControllerPrivate::OnCmd, this->dataPtr.get());
    ignmsg << "LightBarController subscribing to int32 messages on [" << topic << "]" << std::endl;
}

void LightBarController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                             ignition::gazebo::EntityComponentManager &_ecm)
{
    if(_info.paused){
        return;
    }
    if(!this->dataPtr->isInit){
        this->dataPtr->Init(_ecm);
        this->dataPtr->isInit = true;
    }
    // Wait a few frames after init so the renderer finishes creating all
    // visuals before we touch Material components.  This prevents the
    // "Another item already exists" crash in ign-gazebo-6's SceneManager.
    if(this->dataPtr->initFrameCount < LightBarControllerPrivate::kInitDelayFrames){
        ++(this->dataPtr->initFrameCount);
        return;
    }
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->targetMutex);
        //
        if(this->dataPtr->isDone && this->dataPtr->change){
            auto targetMaterial = GetMaterial(this->dataPtr->targetState);
            for(auto &info: this->dataPtr->visualEntityInfos){
                // update SDF of the visual and apply material in-place to avoid
                // remove/create race that causes renderer duplicate-name crash
                info.visualSdf.SetMaterial(targetMaterial);
                const sdf::Material * matPtr = info.visualSdf.Material();
                if(matPtr && info.entity != kNullEntity){
                    if(_ecm.Component<components::Material>(info.entity)){
                        _ecm.Component<components::Material>(info.entity)->Data() = *matPtr;
                    } else {
                        _ecm.CreateComponent(info.entity, components::Material(*matPtr));
                    }
                }
                // mark done (no remove/create needed)
                info.state = 2;
            }
            this->dataPtr->change = false; 
            this->dataPtr->isDone = true;
        }
    }
    if(!this->dataPtr->isDone){
        this->dataPtr->UpdateVisualEnitiies(_ecm);
        // check
        bool flag = true;
        for(auto &info: this->dataPtr->visualEntityInfos){
            if(info.state<2){
                flag = false;
                break;
            }
        }
        this->dataPtr->isDone = flag;
    }
}


/******************implementation for LightBarControllerPrivate******************/
void LightBarControllerPrivate::OnCmd(const ignition::msgs::Int32 &_msg)
{
    std::lock_guard<std::mutex> lock(this->targetMutex);
    this->targetState = _msg.data();
    this->change = true;
}

void LightBarControllerPrivate::Init(ignition::gazebo::EntityComponentManager &_ecm){
    bool flag = false;
    for(auto linkVisual : this->linkVisuals){
        flag = false;
        auto v = common::split(linkVisual,"/");
        if(v.size() == 2){
            auto link = this->model.LinkByName(_ecm, v[0]);
            auto visual = _ecm.EntityByComponents(components::ParentEntity(link),components::Name(v[1]),components::Visual());
            if(visual != kNullEntity){
                sdf::Visual visualSdf = *(this->modelSdf.LinkByName(v[0])->VisualByName(v[1]));
                this->visualEntityInfos.emplace_back(visual,link,visualSdf,2);
                flag = true;
            }
        }
        if(!flag){
            ignerr << "LightBarController: visual element of link [" << linkVisual << "] is invaild" << std::endl;
        }
    }
}

void LightBarControllerPrivate::UpdateVisualEnitiies(ignition::gazebo::EntityComponentManager &_ecm){
    // Simplified and safe behavior: update existing visual's material in-place.
    // Avoid remove/create cycle which can race with the renderer and cause
    // duplicate visual-name crashes.
    for(auto &info: this->visualEntityInfos){
        const sdf::Material * matPtr = info.visualSdf.Material();
        if(matPtr){
            // prefer the stored entity if valid
            if(info.entity != kNullEntity){
                if(_ecm.Component<components::Material>(info.entity)){
                    _ecm.Component<components::Material>(info.entity)->Data() = *matPtr;
                } else {
                    _ecm.CreateComponent(info.entity, components::Material(*matPtr));
                }
            } else {
                // try to locate the visual entity in the ECM and update it
                auto existing = _ecm.EntityByComponents(components::ParentEntity(info.parentEntity),
                                                       components::Name(info.visualSdf.Name()),
                                                       components::Visual());
                if(existing != kNullEntity){
                    info.entity = existing;
                    if(_ecm.Component<components::Material>(existing)){
                        _ecm.Component<components::Material>(existing)->Data() = *matPtr;
                    } else {
                        _ecm.CreateComponent(existing, components::Material(*matPtr));
                    }
                }
            }
        }
        // mark as done
        info.state = 2;
    }
}

/******************register*************************************************/
IGNITION_ADD_PLUGIN(LightBarController,
                    ignition::gazebo::System,
                    LightBarController::ISystemConfigure,
                    LightBarController::ISystemPreUpdate
                    )

IGNITION_ADD_PLUGIN_ALIAS(LightBarController, "ignition::gazebo::systems::LightBarController")
